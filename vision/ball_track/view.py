"""Show the ball_track picture from the ESP32 USB serial.

Close the Arduino Serial Monitor first, or the port is busy.
The STM32 stays on GPIO13 and does not use this port.

    python vision/ball_track/view.py --port COM7

q closes the windows. The mask window is white on pixels the tracker treats as the orange ball.
"""

from __future__ import annotations

import argparse
import math

import cv2
import numpy as np
import serial


MAGIC = b"##FRAME "

# Same HSV window as ball_track.ino.
H_MIN, H_MAX = 0, 25
S_MIN, S_MAX = 29, 255
V_MIN, V_MAX = 142, 255


def parse_args():
    parser = argparse.ArgumentParser(description="ESP32 ball tracker picture")
    parser.add_argument("--port", required=True, help="COM port from Arduino Tools, Port")
    parser.add_argument("--baud", type=int, default=1500000)
    return parser.parse_args()


def later_frame_ready(buf: bytearray, offset: int):
    start = buf.find(MAGIC, offset)
    if start < 0:
        return False
    newline = buf.find(b"\n", start + len(MAGIC))
    if newline < 0:
        return False
    try:
        length = int(buf[start + len(MAGIC) : newline].split()[0])
    except (ValueError, IndexError):
        return True
    if not 500 <= length <= 80000:
        return True
    return len(buf) >= newline + 1 + length


def take_frame(buf: bytearray):
    while True:
        start = buf.find(MAGIC)
        if start < 0:
            if len(buf) > 64:
                del buf[:-64]
            return None
        if start > 0:
            del buf[:start]

        newline = buf.find(b"\n", len(MAGIC))
        if newline < 0:
            return None

        try:
            parts = [int(part) for part in buf[len(MAGIC) : newline].split()]
            if len(parts) == 5:
                length, crc, x, y, count = parts
                diameter = 0
            elif len(parts) == 6:
                length, crc, x, y, count, diameter = parts
            else:
                raise ValueError
        except ValueError:
            del buf[: len(MAGIC)]
            continue

        if not 500 <= length <= 80000:
            del buf[: len(MAGIC)]
            continue

        payload = newline + 1
        end = payload + length
        if len(buf) < end:
            return None
        if later_frame_ready(buf, end):
            del buf[:end]
            continue

        jpg = bytes(buf[payload:end])
        del buf[:end]
        if jpg[:2] != b"\xff\xd8" or jpg[-2:] != b"\xff\xd9":
            continue
        total = int(np.frombuffer(jpg, dtype=np.uint8).sum(dtype=np.uint64)) & 0xFFFFFFFF
        if total != crc:
            continue

        image = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            continue
        return image, x, y, count, diameter


def lift_corners(hsv):
    """The lens dims the corners. Raise value there so the same V min still matches the ball."""
    height, width = hsv.shape[:2]
    y, x = np.ogrid[0:height, 0:width]
    dx = (x - (width - 1) / 2) / ((width - 1) / 2)
    dy = (y - (height - 1) / 2) / ((height - 1) / 2)
    dist = np.sqrt(dx * dx + dy * dy)
    gain = 1.0 + 0.5 * np.minimum(dist, 1.0)
    lifted = hsv.copy()
    value = lifted[:, :, 2].astype(np.float32) * gain
    lifted[:, :, 2] = np.clip(value, 0, 255).astype(np.uint8)
    return lifted


def ball_mask(image):
    """Fast orange preview. Roundness is decided on the ESP32, not in this window."""
    hsv = lift_corners(cv2.cvtColor(image, cv2.COLOR_BGR2HSV))
    lower = np.array([H_MIN, S_MIN, V_MIN], np.uint8)
    upper = np.array([H_MAX, S_MAX, V_MAX], np.uint8)
    mask = cv2.inRange(hsv, lower, upper)
    kernel = np.ones((3, 3), np.uint8)
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)


def draw_label(image, text, x, y):
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(image, text, (x, y), font, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(image, text, (x, y), font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)


def draw_overlay(image, x, y, count, diameter, scale):
    height, width = image.shape[:2]
    if diameter > 0:
        radius = max(4 * scale, int((diameter / 2 + 3) * scale))
    else:
        radius = max(4 * scale, int(math.sqrt(max(count, 0) / math.pi) * scale))
    center = (min(max(int(x * scale), 0), width - 1), min(max(int(y * scale), 0), height - 1))
    cv2.circle(image, center, radius, (0, 255, 0), 2)
    arm = 8 * scale
    cv2.line(image, (center[0] - arm, center[1]), (center[0] + arm, center[1]), (0, 255, 255), 2)
    cv2.line(image, (center[0], center[1] - arm), (center[0], center[1] + arm), (0, 255, 255), 2)

    font = cv2.FONT_HERSHEY_SIMPLEX
    count_text = str(count)
    xy_text = f"{x} {y}"
    (count_w, count_h), _ = cv2.getTextSize(count_text, font, 0.5, 1)
    (xy_w, xy_h), _ = cv2.getTextSize(xy_text, font, 0.5, 1)
    count_x = min(max(center[0] - count_w // 2, 0), width - count_w)
    count_y = center[1] - radius - 6
    if count_y < count_h:
        count_y = count_h
    xy_x = min(max(center[0] - xy_w // 2, 0), width - xy_w)
    xy_y = center[1] + radius + xy_h + 6
    if xy_y > height - 2:
        xy_y = height - 2
    draw_label(image, count_text, count_x, count_y)
    draw_label(image, xy_text, xy_x, xy_y)


def main():
    args = parse_args()
    port = serial.Serial(args.port, args.baud, timeout=0)
    try:
        port.set_buffer_size(rx_size=262144, tx_size=4096)
    except (AttributeError, OSError, serial.SerialException):
        pass
    port.reset_input_buffer()
    buf = bytearray()
    waiting = np.zeros((480, 640, 3), np.uint8)
    cv2.putText(waiting, "waiting for camera", (40, 240), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.namedWindow("ball")
    cv2.namedWindow("mask")
    cv2.imshow("ball", waiting)
    cv2.imshow("mask", np.zeros((480, 640), np.uint8))

    try:
        while True:
            pending = port.in_waiting
            if pending:
                buf.extend(port.read(pending))
            latest = None
            while True:
                got = take_frame(buf)
                if got is None:
                    break
                latest = got
            if latest is not None:
                image, x, y, count, diameter = latest
                show = cv2.resize(image, (image.shape[1] * 2, image.shape[0] * 2), interpolation=cv2.INTER_LINEAR)
                mask = cv2.resize(ball_mask(image), (image.shape[1] * 2, image.shape[0] * 2), interpolation=cv2.INTER_NEAREST)
                draw_overlay(show, x, y, count, diameter, 2)
                cv2.imshow("ball", show)
                cv2.imshow("mask", mask)
            if (cv2.waitKey(1) & 0xFF) == ord("q"):
                break
    finally:
        port.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
