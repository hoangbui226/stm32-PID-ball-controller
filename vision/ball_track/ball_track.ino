#include <Arduino.h>
#include <atomic>
#include <string.h>
#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "jpeg_decoder.h"
#include "board_config.h"

// U0T / USB serial: the camera's own JPEG at 1500000 for view.py.
// GPIO13: "x y count" at 115200 to STM32 PA10 (D2).
// Orange is the first cut. Only a round blob near the predicted position is the ball.
static const int kPictureBaud = 1500000;
static const int kStm32Baud = 115200;
static const int kStm32TxPin = 13;
static const int kJpegQuality = 18;
static const size_t kJobBytes = 80000;
static const size_t kRgbBytes = 320 * 240 * 2;
static const size_t kGrayBytes = 320 * 240;

static const unsigned long kHoldMs = 3000;
static const int kSearchRadius = 80;
static const int kMaxVel = 40;
static const int kMinRadius = 8;
static const int kMaxRadius = 110;
static const int kMinArea = 80;
static const int kSeedCount = 8192;
static const int kCenterX = 160;
static const int kCenterY = 130;
static const int kFullSearchMisses = 3;

static const int h_min = 0;
static const int h_max = 25;
static const int s_min = 29;
static const int s_max = 255;
static const int v_min = 142;
static const int v_max = 255;
static const int s_min_track = 18;
static const int v_min_track = 100;

static std::atomic<int> pub_x{kCenterX};
static std::atomic<int> pub_y{kCenterY};
static std::atomic<int> pub_count{0};
static std::atomic<int> pub_d{0};

struct Seed {
  int16_t x;
  int16_t y;
};

struct Blob {
  int area;
  int perim;
  int32_t sum_x;
  int32_t sum_y;
  int min_x;
  int min_y;
  int max_x;
  int max_y;
};

static uint8_t *job_jpg = nullptr;
static uint8_t *out_jpg = nullptr;
static size_t out_len = 0;
static int out_x = kCenterX;
static int out_y = kCenterY;
static int out_count = 0;
static int out_d = 0;
static std::atomic<bool> out_ready{false};
static uint8_t *rgb = nullptr;
static uint8_t *lift_half = nullptr;
static uint8_t *mask = nullptr;
static uint8_t *work = nullptr;
static Seed *seeds = nullptr;
static int lift_width = 0;
static int lift_height = 0;
static size_t job_len = 0;
static std::atomic<bool> job_ready{false};
static TaskHandle_t detect_task = nullptr;
static uint8_t jpeg_work[3100];

static int isqrt(int n) {
  int x;
  int y;
  if (n <= 0) {
    return 0;
  }
  x = n;
  y = (x + 1) / 2;
  while (y < x) {
    x = y;
    y = (x + n / x) / 2;
  }
  return x;
}

static int clampi(int v, int lo, int hi) {
  if (v < lo) {
    return lo;
  }
  if (v > hi) {
    return hi;
  }
  return v;
}

static int lifted_value(int v, int dist) {
  if (dist > 256) {
    dist = 256;
  }
  v = v * (256 + dist / 2) / 256;
  if (v > 255) {
    v = 255;
  }
  return v;
}

static bool ball_color(int r, int g, int b, int dist) {
  int maxv = r;
  if (g > maxv) maxv = g;
  if (b > maxv) maxv = b;
  int minv = r;
  if (g < minv) minv = g;
  if (b < minv) minv = b;

  int v = lifted_value(maxv, dist);
  int diff = maxv - minv;
  int s = maxv == 0 ? 0 : diff * 255 / maxv;
  int h = 0;
  if (diff > 0) {
    if (maxv == r) {
      h = (g - b) * 30 / diff;
      if (h < 0) h += 180;
    } else if (maxv == g) {
      h = 60 + (b - r) * 30 / diff;
    } else {
      h = 120 + (r - g) * 30 / diff;
    }
  }

  return h >= h_min && h <= h_max && s >= s_min && s <= s_max && v >= v_min && v <= v_max;
}

static bool ball_color_at(int r, int g, int b, int dist, bool tracking) {
  if (!tracking) {
    return ball_color(r, g, b, dist);
  }
  int maxv = r;
  if (g > maxv) maxv = g;
  if (b > maxv) maxv = b;
  int minv = r;
  if (g < minv) minv = g;
  if (b < minv) minv = b;
  int v = lifted_value(maxv, dist);
  int diff = maxv - minv;
  int s = maxv == 0 ? 0 : diff * 255 / maxv;
  int h = 0;
  if (diff > 0) {
    if (maxv == r) {
      h = (g - b) * 30 / diff;
      if (h < 0) h += 180;
    } else if (maxv == g) {
      h = 60 + (b - r) * 30 / diff;
    } else {
      h = 120 + (r - g) * 30 / diff;
    }
  }
  return h >= h_min && h <= h_max && s >= s_min_track && s <= s_max && v >= v_min_track && v <= v_max;
}

static void store_pixel(const uint8_t *p, int *r, int *g, int *b) {
  uint16_t pix = ((uint16_t)p[0] << 8) | p[1];
  *r = ((pix >> 11) & 0x1F) << 3;
  *g = ((pix >> 5) & 0x3F) << 2;
  *b = (pix & 0x1F) << 3;
}

static void erode3(const uint8_t *src, uint8_t *dst, int width, int height, int x0, int y0, int x1, int y1) {
  memset(dst, 0, (size_t)width * (size_t)height);
  int xa = x0 > 1 ? x0 : 1;
  int ya = y0 > 1 ? y0 : 1;
  int xb = x1 < width - 1 ? x1 : width - 1;
  int yb = y1 < height - 1 ? y1 : height - 1;
  for (int y = ya; y < yb; y++) {
    for (int x = xa; x < xb; x++) {
      if (src[(y - 1) * width + (x - 1)] && src[(y - 1) * width + x] && src[(y - 1) * width + (x + 1)] &&
          src[y * width + (x - 1)] && src[y * width + x] && src[y * width + (x + 1)] &&
          src[(y + 1) * width + (x - 1)] && src[(y + 1) * width + x] && src[(y + 1) * width + (x + 1)]) {
        dst[y * width + x] = 1;
      }
    }
  }
}

static void dilate3(const uint8_t *src, uint8_t *dst, int width, int height, int x0, int y0, int x1, int y1) {
  memset(dst, 0, (size_t)width * (size_t)height);
  int xa = x0 > 1 ? x0 : 1;
  int ya = y0 > 1 ? y0 : 1;
  int xb = x1 < width - 1 ? x1 : width - 1;
  int yb = y1 < height - 1 ? y1 : height - 1;
  for (int y = ya; y < yb; y++) {
    for (int x = xa; x < xb; x++) {
      if (src[(y - 1) * width + (x - 1)] || src[(y - 1) * width + x] || src[(y - 1) * width + (x + 1)] ||
          src[y * width + (x - 1)] || src[y * width + x] || src[y * width + (x + 1)] ||
          src[(y + 1) * width + (x - 1)] || src[(y + 1) * width + x] || src[(y + 1) * width + (x + 1)]) {
        dst[y * width + x] = 1;
      }
    }
  }
}

static void clear_blob(Blob *blob, int width, int height) {
  blob->area = 0;
  blob->perim = 0;
  blob->sum_x = 0;
  blob->sum_y = 0;
  blob->min_x = width;
  blob->min_y = height;
  blob->max_x = 0;
  blob->max_y = 0;
}

static void fill_blob(int x, int y, int width, int height, Blob *blob) {
  int count = 0;
  seeds[count].x = (int16_t)x;
  seeds[count].y = (int16_t)y;
  count++;
  while (count > 0) {
    int cx;
    int cy;
    int x1;
    bool up;
    bool down;
    count--;
    cx = seeds[count].x;
    cy = seeds[count].y;
    if (mask[cy * width + cx] != 1) {
      continue;
    }
    x1 = cx;
    while (x1 > 0 && mask[cy * width + (x1 - 1)] == 1) {
      x1--;
    }
    up = false;
    down = false;
    while (x1 < width && mask[cy * width + x1] == 1) {
      int idx = cy * width + x1;
      mask[idx] = 2;
      blob->area++;
      blob->sum_x += x1;
      blob->sum_y += cy;
      if (x1 < blob->min_x) {
        blob->min_x = x1;
      }
      if (x1 > blob->max_x) {
        blob->max_x = x1;
      }
      if (cy < blob->min_y) {
        blob->min_y = cy;
      }
      if (cy > blob->max_y) {
        blob->max_y = cy;
      }
      if (x1 == 0 || cy == 0 || x1 == width - 1 || cy == height - 1 ||
          mask[cy * width + (x1 - 1)] == 0 || mask[cy * width + (x1 + 1)] == 0 ||
          mask[(cy - 1) * width + x1] == 0 || mask[(cy + 1) * width + x1] == 0) {
        blob->perim++;
      }
      if (cy > 0) {
        if (mask[(cy - 1) * width + x1] == 1) {
          if (!up && count < kSeedCount) {
            seeds[count].x = (int16_t)x1;
            seeds[count].y = (int16_t)(cy - 1);
            count++;
            up = true;
          }
        } else {
          up = false;
        }
      }
      if (cy + 1 < height) {
        if (mask[(cy + 1) * width + x1] == 1) {
          if (!down && count < kSeedCount) {
            seeds[count].x = (int16_t)x1;
            seeds[count].y = (int16_t)(cy + 1);
            count++;
            down = true;
          }
        } else {
          down = false;
        }
      }
      x1++;
    }
  }
}

// Acquire is strict so a jumper is not the ball. Tracking is looser so a clipped rim still counts.
static bool round_ball(const Blob *blob, int width, int height, bool tracking, int *cx, int *cy, int *diameter) {
  int w;
  int h;
  int longer;
  int shorter;
  int radius;
  int min_r = kMinRadius * width / 320;
  int max_r = kMaxRadius * width / 320;
  int min_a = kMinArea * width * height / (320 * 240);
  if (min_r < 4) {
    min_r = 4;
  }
  if (min_a < 20) {
    min_a = 20;
  }
  if (tracking && min_a > 12) {
    min_a = min_a / 2;
  }
  if (blob->area < min_a || blob->perim < 8) {
    return false;
  }
  if (!tracking &&
      (blob->min_x == 0 || blob->min_y == 0 || blob->max_x == width - 1 || blob->max_y == height - 1)) {
    return false;
  }
  w = blob->max_x - blob->min_x + 1;
  h = blob->max_y - blob->min_y + 1;
  radius = (w > h ? w : h) / 2;
  if (radius < min_r || radius > max_r) {
    return false;
  }
  longer = w > h ? w : h;
  shorter = w < h ? w : h;
  if (tracking) {
    if (longer > shorter * 2) {
      return false;
    }
    if (blob->area * 3 < w * h) {
      return false;
    }
    if (314 * blob->area < 10 * blob->perim * blob->perim) {
      return false;
    }
  } else {
    if (longer * 2 > shorter * 3) {
      return false;
    }
    if (blob->area * 2 < w * h) {
      return false;
    }
    if (314 * blob->area < 15 * blob->perim * blob->perim) {
      return false;
    }
  }
  *cx = (int)(blob->sum_x / blob->area);
  *cy = (int)(blob->sum_y / blob->area);
  *diameter = (w + h) / 2;
  return true;
}

static void paint_mask(int width, int height, int x0, int y0, int x1, int y1, bool tracking) {
  memset(mask, 0, (size_t)width * (size_t)height);
  for (int y = y0; y < y1; y++) {
    const uint8_t *pix = rgb + (y * width + x0) * 2;
    for (int x = x0; x < x1; x++) {
      int r, g, b;
      int dist = (int)lift_half[y * width + x] * 2;
      store_pixel(pix, &r, &g, &b);
      if (ball_color_at(r, g, b, dist, tracking)) {
        mask[y * width + x] = 1;
      }
      pix += 2;
    }
  }
  erode3(mask, work, width, height, x0, y0, x1, y1);
  dilate3(work, mask, width, height, x0, y0, x1, y1);
}

static bool find_round(int width, int height, int x0, int y0, int x1, int y1, int pred_x, int pred_y,
                      bool nearest, bool tracking, int *out_x, int *out_y, int *out_d, int *out_area) {
  int best_d2 = 0x7fffffff;
  int best_area = 0;
  bool found = false;
  paint_mask(width, height, x0, y0, x1, y1, tracking);
  for (int y = y0; y < y1; y++) {
    for (int x = x0; x < x1; x++) {
      Blob blob;
      int cx;
      int cy;
      int diameter;
      int dx;
      int dy;
      int d2;
      if (mask[y * width + x] != 1) {
        continue;
      }
      clear_blob(&blob, width, height);
      fill_blob(x, y, width, height, &blob);
      if (!round_ball(&blob, width, height, tracking, &cx, &cy, &diameter)) {
        continue;
      }
      dx = cx - pred_x;
      dy = cy - pred_y;
      d2 = dx * dx + dy * dy;
      if (nearest) {
        if (d2 < best_d2) {
          best_d2 = d2;
          *out_x = cx;
          *out_y = cy;
          *out_d = diameter;
          *out_area = blob.area;
          found = true;
        }
      } else if (blob.area > best_area) {
        best_area = blob.area;
        *out_x = cx;
        *out_y = cy;
        *out_d = diameter;
        *out_area = blob.area;
        found = true;
      }
    }
  }
  return found;
}

static bool start_camera() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_QVGA;
  config.pixel_format = PIXFORMAT_JPEG;
  config.jpeg_quality = kJpegQuality;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.fb_count = 2;

  if (!psramFound() || esp_camera_init(&config) != ESP_OK) {
    return false;
  }

  sensor_t *sensor = esp_camera_sensor_get();
  sensor->set_framesize(sensor, FRAMESIZE_QVGA);
  sensor->set_quality(sensor, kJpegQuality);
  sensor->set_brightness(sensor, -1);
  sensor->set_contrast(sensor, 1);
  sensor->set_saturation(sensor, 0);
  sensor->set_whitebal(sensor, 1);
  sensor->set_awb_gain(sensor, 1);
  sensor->set_wb_mode(sensor, 0);
  sensor->set_exposure_ctrl(sensor, 1);
  sensor->set_aec2(sensor, 1);
  sensor->set_gain_ctrl(sensor, 1);
  return true;
}

static uint32_t checksum(const uint8_t *data, size_t len) {
  uint32_t sum = 0;
  for (size_t i = 0; i < len; i++) {
    sum += data[i];
  }
  return sum;
}

static void build_lift(int width, int height) {
  if (lift_half == nullptr || (lift_width == width && lift_height == height)) {
    return;
  }
  int x_span = width - 1;
  int y_span = height - 1;
  for (int y = 0; y < height; y++) {
    int dy = (2 * y - y_span) * 256 / y_span;
    int dy_sq = dy * dy;
    for (int x = 0; x < width; x++) {
      int dx = (2 * x - x_span) * 256 / x_span;
      int dist = isqrt(dx * dx + dy_sq);
      if (dist > 256) {
        dist = 256;
      }
      lift_half[y * width + x] = (uint8_t)(dist / 2);
    }
  }
  lift_width = width;
  lift_height = height;
}

static void downsample2(uint8_t *buf, int src_w, int src_h, int *dst_w, int *dst_h) {
  int dw = src_w / 2;
  int dh = src_h / 2;
  if (dw < 8 || dh < 8) {
    *dst_w = src_w;
    *dst_h = src_h;
    return;
  }
  for (int y = 0; y < dh; y++) {
    const uint8_t *src = buf + (size_t)(2 * y * src_w) * 2;
    uint8_t *dst = buf + (size_t)(y * dw) * 2;
    for (int x = 0; x < dw; x++) {
      dst[0] = src[0];
      dst[1] = src[1];
      dst += 2;
      src += 4;
    }
  }
  *dst_w = dw;
  *dst_h = dh;
}

static void send_picture(const uint8_t *jpg, size_t jpg_len, int x, int y, int count, int diameter) {
  uint32_t sum = checksum(jpg, jpg_len);
  Serial.printf("##FRAME %u %u %d %d %d %d\n", (unsigned)jpg_len, sum, x, y, count, diameter);
  Serial.write(jpg, jpg_len);
}

static void track_jpeg(const uint8_t *jpg, size_t len) {
  static int center_x = 80;
  static int center_y = 65;
  static int center_d = 0;
  static int center_area = 0;
  static int vel_x = 0;
  static int vel_y = 0;
  static int miss_count = 0;
  static unsigned long lost_ms = 0;
  static bool ball_lost = false;
  static bool have_lock = false;

  esp_jpeg_image_cfg_t cfg;
  esp_jpeg_image_output_t out;
  int width;
  int height;
  int scale_x;
  int scale_y;
  int search;
  int pred_x;
  int pred_y;
  int x0;
  int y0;
  int x1;
  int y1;
  int pick_x = 0;
  int pick_y = 0;
  int pick_d = 0;
  int pick_area = 0;
  bool found = false;

  memset(&cfg, 0, sizeof(cfg));
  cfg.indata = (uint8_t *)jpg;
  cfg.indata_size = len;
  cfg.outbuf = rgb;
  cfg.outbuf_size = kRgbBytes;
  cfg.out_format = JPEG_IMAGE_FORMAT_RGB565;
  cfg.out_scale = JPEG_IMAGE_SCALE_0;
  cfg.flags.swap_color_bytes = 1;
  cfg.advanced.working_buffer = jpeg_work;
  cfg.advanced.working_buffer_size = sizeof(jpeg_work);
  memset(&out, 0, sizeof(out));
  if (rgb == nullptr || lift_half == nullptr || mask == nullptr || work == nullptr || seeds == nullptr ||
      esp_jpeg_decode(&cfg, &out) != ESP_OK) {
    return;
  }
  width = out.width;
  height = out.height;
  if (width < 8 || height < 8 || width > 320 || height > 240) {
    return;
  }
  downsample2(rgb, width, height, &width, &height);
  build_lift(width, height);
  scale_x = 320 / width;
  scale_y = 240 / height;
  if (scale_x < 1) {
    scale_x = 1;
  }
  if (scale_y < 1) {
    scale_y = 1;
  }
  search = kSearchRadius / scale_x;
  if (search < 24) {
    search = 24;
  }

  pred_x = clampi(center_x + vel_x, 0, width - 1);
  pred_y = clampi(center_y + vel_y, 0, height - 1);

  if (have_lock) {
    x0 = clampi(pred_x - search, 0, width);
    y0 = clampi(pred_y - search, 0, height);
    x1 = clampi(pred_x + search, 0, width);
    y1 = clampi(pred_y + search, 0, height);
    found = find_round(width, height, x0, y0, x1, y1, pred_x, pred_y, true, true, &pick_x, &pick_y, &pick_d,
                       &pick_area);
  }
  if (!found && (!have_lock || miss_count + 1 >= kFullSearchMisses)) {
    found = find_round(width, height, 0, 0, width, height, pred_x, pred_y, have_lock, false, &pick_x, &pick_y, &pick_d,
                       &pick_area);
  }

  if (found) {
    vel_x = clampi(pick_x - center_x, -kMaxVel / scale_x, kMaxVel / scale_x);
    vel_y = clampi(pick_y - center_y, -kMaxVel / scale_y, kMaxVel / scale_y);
    center_x = pick_x;
    center_y = pick_y;
    center_d = pick_d;
    center_area = pick_area;
    have_lock = true;
    ball_lost = false;
    miss_count = 0;
  } else if (!ball_lost) {
    lost_ms = millis();
    ball_lost = true;
    vel_x = 0;
    vel_y = 0;
    miss_count++;
  } else if ((unsigned long)(millis() - lost_ms) >= kHoldMs) {
    center_x = kCenterX * width / 320;
    center_y = kCenterY * height / 240;
    center_d = 0;
    center_area = 0;
    vel_x = 0;
    vel_y = 0;
    miss_count = 0;
    have_lock = false;
  } else {
    miss_count++;
  }

  pub_x.store(center_x * scale_x, std::memory_order_relaxed);
  pub_y.store(center_y * scale_y, std::memory_order_relaxed);
  pub_count.store(center_area * scale_x * scale_y, std::memory_order_relaxed);
  pub_d.store(center_d * scale_x, std::memory_order_relaxed);
}

static void publish_picture() {
  if (out_ready.load(std::memory_order_acquire) || out_jpg == nullptr) {
    return;
  }
  uint8_t *swap = out_jpg;
  out_jpg = job_jpg;
  job_jpg = swap;
  out_len = job_len;
  out_x = pub_x.load(std::memory_order_relaxed);
  out_y = pub_y.load(std::memory_order_relaxed);
  out_count = pub_count.load(std::memory_order_relaxed);
  out_d = pub_d.load(std::memory_order_relaxed);
  out_ready.store(true, std::memory_order_release);
}

static void detect_loop(void *) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (job_ready.load(std::memory_order_acquire)) {
      track_jpeg(job_jpg, job_len);
      publish_picture();
      job_ready.store(false, std::memory_order_release);
    }
  }
}

static bool offer_frame(const uint8_t *jpg, size_t len) {
  if (detect_task == nullptr || job_jpg == nullptr || len > kJobBytes) {
    return false;
  }
  if (job_ready.load(std::memory_order_acquire)) {
    return false;
  }
  memcpy(job_jpg, jpg, len);
  job_len = len;
  job_ready.store(true, std::memory_order_release);
  xTaskNotifyGive(detect_task);
  return true;
}

void setup() {
  Serial.setTxBufferSize(8192);
  Serial.begin(kPictureBaud);
  Serial1.begin(kStm32Baud, SERIAL_8N1, -1, kStm32TxPin);
  if (!start_camera()) {
    while (true) {
      delay(1000);
    }
  }

  job_jpg = (uint8_t *)heap_caps_malloc(kJobBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  out_jpg = (uint8_t *)heap_caps_malloc(kJobBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  rgb = (uint8_t *)heap_caps_malloc(kRgbBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  lift_half = (uint8_t *)heap_caps_malloc(kGrayBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  mask = (uint8_t *)heap_caps_malloc(kGrayBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  work = (uint8_t *)heap_caps_malloc(kGrayBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  seeds = (Seed *)heap_caps_malloc(kSeedCount * sizeof(Seed), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (job_jpg != nullptr && out_jpg != nullptr && rgb != nullptr && lift_half != nullptr && mask != nullptr &&
      work != nullptr && seeds != nullptr) {
    xTaskCreatePinnedToCore(detect_loop, "detect", 16384, nullptr, 2, &detect_task, 0);
  }
}

void loop() {
  camera_fb_t *fb = esp_camera_fb_get();
  int x;
  int y;
  int count;
  int diameter;
  if (fb == nullptr || fb->format != PIXFORMAT_JPEG) {
    if (fb != nullptr) {
      esp_camera_fb_return(fb);
    }
    return;
  }

  offer_frame(fb->buf, fb->len);
  x = pub_x.load(std::memory_order_relaxed);
  y = pub_y.load(std::memory_order_relaxed);
  count = pub_count.load(std::memory_order_relaxed);
  diameter = pub_d.load(std::memory_order_relaxed);
  Serial1.printf("%d %d %d\n", x, y, count);
  if (out_ready.load(std::memory_order_acquire) && out_jpg != nullptr) {
    send_picture(out_jpg, out_len, out_x, out_y, out_count, out_d);
    out_ready.store(false, std::memory_order_release);
  }
  esp_camera_fb_return(fb);
}
