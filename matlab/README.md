# MATLAB ball-balancer simulator

Mirrors the STM32 pipeline so you can see **every** intermediate variable while tuning.

```
plate ball (cm)
   -> fake camera pixels
   -> EMA + deadband          (main.c)
   -> err_x, err_y
   -> PID P/I/D + LPF         (pid.c)  -> theta, phi
   -> optional 180° plant flip
   -> IK                      (ik.c)   -> deg1, deg2, deg3
   -> ik_to_servo             (main.c) -> servo A/B/C
   -> gravity plant on tilted plate
```

## Run

Open **MATLAB** (not PowerShell), `cd` into this `matlab` folder, then:

```matlab
simulate_ball_robot   % offline sim + plots + 3D replay
live_ball_robot       % click / buttons to move the ball, arms respond live
geogebra_arm          % 2D GeoGebra-style single-arm sketch (drag B / slider)
```

`geogebra_arm`: drag blue point **B** around **A**, or use the theta slider. Left panel shows live points/angles (EF, FG, GD, beta, alpha) from `ik.c` lengths.

`live_ball_robot`: click the gray plate or use Place/Reset/Quit buttons; `R` reset; `Q` quit.

`simulate_ball_robot` opens plots plus an animated 3D replay. Workspace also gets:

| Name | What it is |
|------|------------|
| `p` | All parameters (`robot_params`) |
| `log` | Full time history of every signal |
| `pid_state` | Final PID integrator / memory |
| `last_pid` | Last PID step internals |
| `last_ik` | Last IK intermediates (`a,b,c`, discs, …) |
| `last_joints` | Last 3D arm points |

Browse signals with:

```matlab
openvar('log')
plot(log.t, log.err_px)
plot(log.t, log.servo)
```

## Tune

Edit gains / timing in `robot_params.m`, then re-run `simulate_ball_robot`.

Important knobs:

- `kp`, `ki`, `kd`, `alpha`, `beta` — same as `pid.c`
- `balance_flip` — `true` makes the gravity plant stabilize with firmware-style phi (recommended for closed-loop tests). Set `false` to see raw firmware tilt direction on the plant.
- `ball0` — starting ball position (cm)

## Signal dictionary (`log.*`)

| Field | Units | Meaning |
|-------|-------|---------|
| `t` | s | Time |
| `ball_cm` | cm | Ball on plate `[x y]` |
| `vel_cm_s` | cm/s | Ball velocity |
| `acc_cm_s2` | cm/s² | In-plane acceleration |
| `ball_px` | px | Ideal camera sample |
| `filt_px` | px | EMA-filtered camera (like `ball_x/y`) |
| `err_px` | px | Error after deadband |
| `sum_err` | px·s | PID integral state |
| `d_err` | px/s | Error derivative |
| `p_term` / `i_term` / `d_term` | — | Separate PID terms |
| `raw` | — | `P+I+D` before LPF |
| `filt` | — | After `alpha` LPF |
| `radius` | — | `hypot(filt)` |
| `theta`, `phi` | deg | Firmware plate command |
| `theta_plant`, `phi_plant` | deg | What the plant uses |
| `deg` | deg | IK horn angles `[deg1 deg2 deg3]` |
| `servo` | deg | PWM commands `[A B C]` → D13/D12/D11 |
| `ik_ok` | 0/1 | IK found a real solution |
| `n_hat` | — | Plate unit normal |

Arm mapping (same as `note.txt`):

- `deg1` / servo A → Arm A (120°), pin D13
- `deg2` / servo B → +X arm, pin D12
- `deg3` / servo C → Arm C (240°), pin D11

## Files

| File | Role |
|------|------|
| `simulate_ball_robot.m` | Offline sim + plots |
| `live_ball_robot.m` | Click-to-place realtime 3D |
| `animate_ball_robot.m` | Replay a `log` in 3D |
| `robot_params.m` | Geometry + gains |
| `pid_step.m` | `pid.c` |
| `ik_solve.m` | `ik.c` |
| `ik_to_servo.m` | `main.c` servo map |
| `robot_draw.m` / `robot_draw_update.m` | 3D artists |
