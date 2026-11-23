# xlerobot-gravity-compensation

Gravity compensation for the two SO-101 arms of the XLeRobot (Feetech STS3215 servos, Jetson Orin). You move the arm by hand and it stays where you leave it.

This is a port of NVIDIA's [Make a robot arm feel weightless with Holoscan 5](https://developer.nvidia.com/blog/make-a-robot-arm-feel-weightless-with-nvidia-holoscan-5/) ([tutorial](https://nvidia-holoscan.github.io/tutorials/rebot-gravity-compensation/)). NVIDIA used a Seeed reBot B601-DM arm. This version is plain Rust with no Holoscan, written for a hobby-servo arm.

## Can LeRobot / XLeRobot do this? (checked 2026-11-23)

Yes, but only approximately. The details:

| NVIDIA reBot setup | XLeRobot | Result |
|---|---|---|
| Quasi-direct-drive motors with a torque command (MIT mode) | STS3215: geared position servo. It has **no current/torque mode**, but it has an open-loop **PWM mode** (`Operating_Mode` = 2, duty in `Goal_Time` @ 44, sign bit 10) | PWM duty is the torque command. Torque ≈ duty × stall torque at low speed. Read on both arms: firmware 3.10, 12.1 V |
| Pinocchio RNEA on the URDF | MakerMods `xlerobot.urdf` with SO-101 arm inertials. Pinocchio is not on the Orin env, so the model is reimplemented in Rust | `make check-model`: Rust gravity equals Pinocchio `computeGeneralizedGravity` to **2e-15 N·m** over 200 random poses per arm |
| 400 Hz Holoscan graph | Serial bus at 1 Mbaud: a 6-motor state read takes **1.16 ms** (measured), and the PWM sync-write needs no reply | 250 Hz default. The bus is the limit, not the scheduler |
| Holoscan 5 SDK | **Cannot be installed on this Orin.** `holoscan==5.0.0` needs `holoscan-core-cu13`, which has no wheel for JetPack 6 / L4T R36.5 / CUDA 12.6. NVIDIA tested on JetPack 7.2 | Not used. The loop is a fixed-rate Rust thread per arm |
| Backdrivable QDD actuators | ~1:345 gearing gives high static friction | It feels **lighter**, not fully weightless. Coulomb-friction feedforward (`friction_pwm`) helps once the joint moves |

LeRobot 0.7 talks to the servos through `rustypot`, a Rust crate. This repo uses the same crate directly, so the register map and EEPROM-lock handling match LeRobot exactly.

## How it works

```
every 4 ms, per arm (own thread, own serial bus):
  sync-read  pos/speed/load of ids 1..6           (1 bus transaction)
  q_urdf   = sign * deg2rad((raw - mid) * 360/4095) + offset     # LeRobot DEGREES -> URDF
  tau      = g(q_urdf)                                           # gravity, URDF tree walk (= RNEA gravity term)
  pwm      = pwm_sign * ( scale*ramp*pwm_per_nm*sign*tau
                        + friction_pwm*tanh(v/friction_vel) + viscous_pwm*v )
             + virtual walls near the calibrated range ends
  sync-write Goal_Time (PWM) to the 5 arm joints  (no reply)
```

The URDF mapping (`sign`, `offset_rad`) comes from `robotics-antigravity-plugin/.../xlerobot_real_plan.py`. That mapping was matched to LeRobot's `so101_new_calib` model. The gripper is not compensated: it stays in position mode and holds the ball.

| NVIDIA operator | Here |
|---|---|
| `JointStateSourceOp` (OnClock 2.5 ms) | `Bus::read_state` at the start of each cycle (`src/servo.rs`) |
| `PinocchioGravityOp` (kp, kd, tau_scale, joint_direction) | `ArmModel::gravity` + per-joint `pwm_per_nm`, `pwm_sign`, `gravity_scale` (`src/model.rs`, `src/control.rs`) |
| `MotorCommandSinkOp`, `stop()` → `robot.disable()` | `Bus::write_pwm`, then `Bus::hold_here` on any exit |
| 100 ms command watchdog | Every fault and every exit path holds the arm in place: speed, range, temperature, voltage, bus errors, Ctrl+C, panic. One arm faulting stops both arms |

### Safety

- On exit or fault, each motor goes back to **position mode holding where it is**, so the arm doesn't drop. `gravcomp release --yes` is the only command that turns torque off.
- Start with `--scale 0.5`. The gravity term fades in over `ramp_s`, and PWM is capped per joint (`max_pwm`, also written to `Torque_Limit`).
- Faults trigger above 4 rad/s, more than 5° outside the calibrated range, above 60 °C, or below 10.5 V. A virtual wall pushes back inside 6° of each range end.
- **A `kill -9` or a power loss mid-run leaves the motors at their last PWM.** Keep the power switch within reach. `make hold` recovers. The mode is written with the EEPROM lock open (the same convention LeRobot uses), so a motor could still be in PWM mode after a power cycle. LeRobot's `configure()` sets position mode again on its next connect.
- The serial ports are exclusive. `make free-ports` stops the xlerobot-ops host or bridge.

## Usage (from the laptop)

```bash
make test check-model        # laptop: unit tests + Pinocchio cross-check (pip install pin)
make rust-orin               # once: user-level rustup on the Orin
make build                   # rsync + cargo build --release on the Orin
make free-ports              # stop xlerobot-ops host/bridge
make probe                   # read-only: angles, model torque, load. Moves nothing
make pose ARM=left           # move to the 'extended' pose (all joints mid-range, forearm forward) and hold
make identify ARM=left JOINT=elbow_flex    # break-away PWM both ways -> pwm_sign, friction_pwm, pwm_per_nm
make run ARM=left JOINTS=elbow_flex SCALE=0.5 POSE=extended   # one joint first
make run ARM=both SCALE=0.8 POSE=extended                     # everything
make demo SCALE=0.8          # pose -> hand over 2 ping pong balls -> gravcomp + 3-camera recording
make fetch-demo              # copy demo/<stamp>/{demo.mp4,frames.csv,left.csv,right.csv} to the laptop
```

Tuning: put the identified values in `config/xlerobot.toml` under `[arms.<arm>.overrides.<joint>]`. Measure `pwm_per_nm` in a pose where the joint carries load: the elbow and wrist in the extended pose, the shoulder_lift with the upper arm tilted forward. Raise `--scale` toward 1.0. Add `friction_pwm` until the joint glides without drifting.

### Demo

`make demo` opens a tmux session with two windows:

1. **record**: `tools/record_cameras.py` writes one 1440×540 video: the head camera large, both wrist cameras stacked beside it, with a time overlay. It also writes `frames.csv` with a unix time per frame.
2. **gravcomp**: moves both arms to `extended`. It opens each gripper and asks you to hand in a ping pong ball, then closes until the servo load rises (`grasp_load`). It keeps the squeeze capped (`hold_torque_limit`) and runs gravity compensation on both arms. Joint logs go to `demo/<stamp>/{left,right}.csv`, on the same clock as the video.

Ctrl+C in the gravcomp window (the arms hold), then Ctrl+C in the record window.

## Layout

```
src/servo.rs     STS3215 PWM-mode access over rustypot (sync read/write, mode switch, hold, release)
src/model.rs     URDF arm tree + gravity torque (validated against Pinocchio)
src/control.rs   control loop, safety, gripper grasp, identification, move-to-pose
src/config.rs    config/xlerobot.toml + LeRobot calibration JSON
src/main.rs      CLI: probe | gravity | pose | identify | run | hold | release
config/xlerobot.toml    joints, mapping, gains, poses, safety limits
urdf/xlerobot.urdf      MakerMods XLeRobot URDF (Apache-2.0, unmodified; LICENCE.txt)
tools/check_gravity_pinocchio.py   Rust vs Pinocchio
tools/record_cameras.py            3-camera demo recorder
```

## Holoscan later

On JetPack 7.x, the three NVIDIA operators map onto this code one-to-one (table above). A Holoscan graph would wrap `read_state`, `gravity` and `write_pwm`. The serial bus would still cap the rate at about 300–400 Hz.
