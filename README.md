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
- Modes are switched **with torque on** and the EEPROM lock stays closed. Turning torque off first, as LeRobot does, dropped the loaded elbow in the first test and the motor stopped answering. With the lock closed, the mode change lives in RAM only, so a power cycle always comes back in position mode. When holding, the goal is set to the present position *before* the mode flips back, so the joint never falls freely.
- **In PWM mode the STS3215 (fw 3.10) reports Present_Position without its homing offset.** Measured on the left arm: the reading jumped by +1164 steps on wrist_roll, where the offset is 1163. `enter_pwm` measures that jump across the mode change, checks it against the offset register, and subtracts it from every PWM-mode reading. `hold_here` writes the goal in the position-mode frame, then rewrites it from a fresh position-mode reading, so no stale goal is left behind.
- **A `kill -9` mid-run leaves the motors at their last PWM.** Keep the power switch within reach. `make hold` recovers.
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
make rest ARM=left           # fold to rest and turn torque off (servos warm up while holding)
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
src/impedance.rs joint / pose impedance, UDP goal + state interface
src/main.rs      CLI: probe | gravity | kin | pose | identify | run [--listen --hold] | hold | release
config/xlerobot.toml    joints, mapping, gains, poses, safety limits
urdf/xlerobot.urdf      MakerMods XLeRobot URDF (Apache-2.0, unmodified; LICENCE.txt)
tools/check_gravity_pinocchio.py   Rust vs Pinocchio
tools/record_cameras.py            3-camera demo recorder
tools/send_goal.py                 impedance goal client (hold, nudge, pose, joints, circle)
```

## Impedance control (goal spring on top of gravity compensation)

This ports the tutorial's second example. A goal pulls the arm like a spring while gravity compensation keeps running, so you can still push the arm aside. Goals can come from anywhere, a learned policy for example, as JSON over UDP: left arm on port 5701, right arm on 5702.

```
joint goal:  tau = Kq (q_d - q) - Dq qd                                  (LeRobot degrees in, like an ACT policy's action)
pose goal:   tau = J^T [ Kp (p_d - p) - Dp v ;  Ko log(R_d R^T) - Do w ] - Dq qd    (tool = Fixed_Jaw_tip, arm base frame)
```

The tool pose and Jacobian match Pinocchio (`getFrameJacobian`, LOCAL_WORLD_ALIGNED) to 3e-15 (`make check-model`).

The SO-101 has only 5 joints, so `J^T` simply ignores the orientation component the arm can't reach. Safety limits:

- Goals are rate-limited: 0.15 m/s, 1 rad/s and 45°/s.
- Pose goals are clamped to the arm's reach, and joint goals to the calibrated range.
- Force (6 N), moment and per-joint spring torque are capped.
- A goal expires after `ttl`, 0.5 s by default. With no new goal the arm goes back to plain gravity compensation and stays compliant.

```bash
make imp ARM=left HOLD=1 POSE=extended                 # soft spring at the extended pose; push it and it springs back
python tools/send_goal.py --arm left state             # tool pose, joints, mode (streams back at 50 Hz)
python tools/send_goal.py --arm left nudge 0 0 0.02    # goal 2 cm up
python tools/send_goal.py --arm left circle --radius 0.03 --period 4
python tools/send_goal.py --arm left joints 0 0 0 0 0  # joint goal
python tools/send_goal.py --arm left off               # gravity only
```

A policy streams `{"mode":"joints","q_deg":[...],"ttl":0.2}` or `{"mode":"pose","pos":[x,y,z],"quat":[w,x,y,z],"ttl":0.2}` once per step, and reads the state stream as its observation.

Limitation: static friction is 0.3–0.6 N·m per joint, so a soft spring has a dead band. At `kp_lin = 150 N/m` that's about 1–2 cm. Stiffer springs, set in `[impedance]` or per goal with `"stiffness"`, tighten it but feel less compliant.

## Holoscan later

On JetPack 7.x, the three NVIDIA operators map onto this code one-to-one (table above). A Holoscan graph would wrap `read_state`, `gravity` and `write_pwm`. The serial bus would still cap the rate at about 300–400 Hz.
