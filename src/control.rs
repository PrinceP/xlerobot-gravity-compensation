//! Gravity-compensation loop for one arm, gripper grasp, and per-joint identification.
//!
//! Per cycle (one sync read + one sync write on the arm's bus):
//!   q      <- Present_Position (5 joints + gripper)
//!   tau    =  g(q) from the URDF model, per joint, in the motor's direction
//!   pwm    =  pwm_sign * (scale * ramp * gravity_scale * pwm_per_nm * tau
//!                         + friction_pwm * tanh(v / friction_vel) + viscous_pwm * v)
//!             + virtual walls near the calibrated range ends
//!   Goal_Time (PWM) <- clamp(pwm, ±max_pwm)
//!
//! Anything unexpected (speed, range, temperature, voltage, bus errors, Ctrl+C, panic)
//! ends the loop; the caller then switches every motor back to position mode holding
//! its present position (`Bus::hold_here`).

use std::fs::File;
use std::io::{BufWriter, Write};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use anyhow::{Result, bail};

use crate::config::{Arm, Config, Joint};
use crate::impedance::Impedance;
use crate::model::ArmModel;
use crate::servo::{self, Bus, RawState};

pub fn unix_s() -> f64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map_or(0.0, |d| d.as_secs_f64())
}

/// Motor raw position -> URDF joint angles (5 joints + jaw).
pub fn urdf_q(arm: &Arm, cfg: &Config, st: &[RawState]) -> Vec<f64> {
    let mut q: Vec<f64> = arm
        .joints
        .iter()
        .zip(st)
        .map(|(j, s)| j.sign * j.cal.raw_to_deg(s.pos).to_radians() + j.offset_rad)
        .collect();
    let g = st.get(arm.joints.len()).map_or(50.0, |s| arm.gripper_cal.raw_to_pct(s.pos));
    q.push(g / 100.0 * cfg.gripper.span_rad);
    q
}

/// Gravity torque in each motor's own direction (N·m), 5 joints.
pub fn motor_gravity(arm: &Arm, model: &ArmModel, q: &[f64]) -> Vec<f64> {
    let g = model.gravity(q);
    arm.joints.iter().zip(&g).map(|(j, t)| j.sign * t).collect()
}

fn wall(j: &Joint, deg: f64, margin: f64, k: f64) -> f64 {
    let (lo, hi) = j.cal.deg_range();
    if deg < lo + margin {
        k * (lo + margin - deg)
    } else if deg > hi - margin {
        -k * (deg - (hi - margin))
    } else {
        0.0
    }
}

pub struct RunOpts {
    pub scale: f64,
    pub duration: Option<f64>,
    pub log: Option<String>,
    pub verbose: bool,
}

pub struct Loop<'a> {
    pub arm: &'a Arm,
    pub cfg: &'a Config,
    pub model: &'a ArmModel,
    pub bus: Bus,
    /// Optional impedance controller (goal spring) on top of gravity compensation.
    pub imp: Option<Impedance>,
}

impl Loop<'_> {
    /// Joints driven in PWM mode (enabled ones). Disabled joints hold position.
    pub fn pwm_joints(&self) -> Vec<usize> {
        (0..self.arm.joints.len()).filter(|&i| self.arm.joints[i].enabled).collect()
    }

    /// Run until stop/fault. Motors are put in PWM mode here; the caller must call
    /// `bus.hold_here` afterwards whatever this returns.
    pub fn run(&mut self, opts: &RunOpts, stop: &Arc<AtomicBool>) -> Result<()> {
        let arm = self.arm;
        let cfg = self.cfg;
        let s = &cfg.safety;
        let tag = format!("[{}]", arm.name);
        let active = self.pwm_joints();
        if active.is_empty() {
            bail!("{tag} no joints enabled");
        }
        let all_ids: Vec<u8> = arm.ids().into_iter().chain([cfg.gripper.id]).collect();
        let pwm_ids: Vec<u8> = active.iter().map(|&i| arm.joints[i].id).collect();
        let limits: Vec<u16> = active.iter().map(|&i| arm.joints[i].max_pwm.round() as u16).collect();

        // Disabled joints: hold where they are.
        let held: Vec<u8> = arm.joints.iter().filter(|j| !j.enabled).map(|j| j.id).collect();
        if !held.is_empty() {
            self.bus.hold_here(&held, s.hold_torque_limit)?;
        }

        // Pre-flight: voltage and temperature.
        for &id in &pwm_ids {
            let v = self.bus.read_u8(id, servo::ADDR_PRESENT_VOLTAGE)? as f64 / 10.0;
            let t = self.bus.read_u8(id, servo::ADDR_PRESENT_TEMPERATURE)?;
            if v < s.min_voltage_v || t > s.max_temp_c {
                bail!("{tag} motor {id}: {v:.1} V, {t} °C outside limits");
            }
        }

        let mut log = match &opts.log {
            Some(p) => {
                let mut w = BufWriter::new(File::create(p)?);
                let mut h = String::from("t_unix,arm");
                for j in &arm.joints {
                    h += &format!(",{0}_deg,{0}_radps,{0}_tau,{0}_pwm,{0}_load", j.name);
                }
                h += ",gripper_pct,gripper_load,cycle_ms,imp_mode,tool_x,tool_y,tool_z,goal_x,goal_y,goal_z,pos_err_mm,rot_err_deg,tool_qw,tool_qx,tool_qy,tool_qz\n";
                w.write_all(h.as_bytes())?;
                Some(w)
            }
            None => None,
        };

        self.bus.enter_pwm(&pwm_ids, &limits)?;
        println!(
            "{tag} PWM mode on {:?}, scale {:.2}, {:.0} Hz. Ctrl+C to hold and exit.",
            active.iter().map(|&i| arm.joints[i].name.as_str()).collect::<Vec<_>>(),
            opts.scale,
            cfg.rate_hz
        );

        let period = Duration::from_secs_f64(1.0 / cfg.rate_hz);
        let t0 = Instant::now();
        let mut next = t0;
        let mut v_f = vec![0.0; arm.joints.len()];
        let mut errors = 0u32;
        let mut n = 0u64;
        let mut worst_ms = 0.0f64;
        let mut last_print = t0;
        let mut rr = 0usize; // round-robin health check
        let mut pwm_out = vec![0i32; active.len()];
        let mut hot = vec![0u32; active.len()];
        let mut still_s = 0.0f64; // time the arm has been still and away from its goal
        // Slow velocity for stillness only: one-step encoder flicker must not reset the timer.
        let mut v_slow = vec![0.0f64; arm.joints.len()];
        let mut low_v = vec![0u32; active.len()];

        while !stop.load(Ordering::Relaxed) {
            let cycle_start = Instant::now();
            let elapsed = (cycle_start - t0).as_secs_f64();
            if opts.duration.is_some_and(|d| elapsed > d) {
                println!("{tag} duration reached");
                break;
            }

            let st = match self.bus.read_state(&all_ids) {
                Ok(st) => {
                    errors = 0;
                    st
                }
                Err(e) => {
                    errors += 1;
                    if errors >= s.max_comm_errors {
                        bail!("{tag} {errors} consecutive bus errors, last: {e:#}");
                    }
                    next += period;
                    std::thread::sleep(next.saturating_duration_since(Instant::now()));
                    continue;
                }
            };

            let q = urdf_q(arm, cfg, &st);
            let tau = motor_gravity(arm, self.model, &q);
            // Fade-in: gravity from ramp_from to 1; friction/viscous terms from 0 to 1 (they
            // would otherwise assist the initial sag).
            let fade = if cfg.ramp_s > 0.0 { (elapsed / cfg.ramp_s).min(1.0) } else { 1.0 };
            let ramp = cfg.ramp_from + (1.0 - cfg.ramp_from) * fade;

            for i in 0..arm.joints.len() {
                v_f[i] = 0.7 * v_f[i] + 0.3 * st[i].speed_radps();
                v_slow[i] = 0.95 * v_slow[i] + 0.05 * st[i].speed_radps();
            }
            // Impedance torque (URDF frame) -> motor frame, faded in like friction.
            let mut imp_tau = vec![0.0; arm.joints.len()];
            let mut imp_status = None;
            let mut imp_log = String::from(",,,,,,,,,,,,,");
            if let Some(imp) = self.imp.as_mut() {
                imp.poll(arm);
                let qd: Vec<f64> = arm.joints.iter().zip(&v_f).map(|(j, v)| j.sign * v).collect();
                let out = imp.torque(self.model, &q, &qd, period.as_secs_f64())?;
                for (i, j) in arm.joints.iter().enumerate() {
                    imp_tau[i] = j.sign * out.tau[i];
                }
                let deg: Vec<f64> = arm.joints.iter().zip(&st).map(|(j, s)| j.cal.raw_to_deg(s.pos)).collect();
                let quat = imp.tool_quat(self.model, &q);
                imp.send_state(arm, &deg, &out, quat);
                imp_status = Some((out.mode_name, out.ee, imp.last_err_m));
                let qt = quat.map_or([f64::NAN; 4], |q| [q.w, q.i, q.j, q.k]);
                imp_log = format!(
                    ",{},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.2},{:.2},{:.5},{:.5},{:.5},{:.5}",
                    out.mode_name, out.ee[0], out.ee[1], out.ee[2], out.goal[0], out.goal[1], out.goal[2],
                    imp.last_err_m * 1e3, imp.last_rot_err_deg, qt[0], qt[1], qt[2], qt[3]
                );
            }

            // Soft push, firm return (see ImpCfg::return_assist).
            let mut assist_eff = cfg.impedance.as_ref().map_or(0.0, |c| c.assist_frac);
            if let (Some(c), Some(imp)) = (cfg.impedance.as_ref(), self.imp.as_mut()) {
                let still = active.iter().all(|&i| v_slow[i].abs() < c.still_radps);
                let away = imp.last_err_m * 1e3 > c.return_err_mm || imp.last_rot_err_deg > c.return_err_deg;
                still_s = if still && away { still_s + period.as_secs_f64() } else { 0.0 };
                if c.return_assist > c.assist_frac {
                    let b = ((still_s - c.return_delay_s) / c.return_ramp_s.max(1e-3)).clamp(0.0, 1.0);
                    assist_eff = c.assist_frac + (c.return_assist - c.assist_frac) * b;
                }
                imp.assist_now = assist_eff;
            }

            for (k, &i) in active.iter().enumerate() {
                let j = &arm.joints[i];
                let deg = j.cal.raw_to_deg(st[i].pos);
                let (lo, hi) = j.cal.deg_range();
                if v_f[i].abs() > s.max_speed_radps {
                    bail!("{tag} {} moving {:.1} rad/s (limit {:.1})", j.name, v_f[i], s.max_speed_radps);
                }
                if deg < lo - 5.0 || deg > hi + 5.0 {
                    bail!("{tag} {} at {deg:.1}° outside calibrated range {lo:.1}..{hi:.1}", j.name);
                }
                // Coulomb feedforward in the direction of motion; at rest, in the direction the
                // impedance spring pulls (breaks stiction toward the goal, fades out near it).
                let mv = if j.friction_vel > 0.0 { (v_f[i] / j.friction_vel).tanh() } else { 0.0 };
                let (frac, tau0) = (assist_eff, cfg.impedance.as_ref().map_or(1.0, |c| c.assist_tau0));
                let intent = (imp_tau[i] / tau0).tanh() * (1.0 - mv.abs());
                let fric = j.friction_pwm * mv + frac * j.stiction_pwm * intent;
                let ff = opts.scale * ramp * j.gravity_scale * j.pwm_per_nm * tau[i]
                    + fade * (j.pwm_per_nm * imp_tau[i] + fric + j.viscous_pwm * v_f[i]);
                let w = wall(j, deg, s.limit_margin_deg, s.wall_pwm_per_deg);
                let pwm = (j.pwm_sign * (ff + w)).clamp(-j.max_pwm, j.max_pwm);
                pwm_out[k] = pwm.round() as i32;
            }

            if let Err(e) = self.bus.write_pwm(&pwm_ids, &pwm_out) {
                errors += 1;
                if errors >= s.max_comm_errors {
                    bail!("{tag} PWM write failing: {e:#}");
                }
            }

            // Health read every 25 cycles, round-robin over the PWM motors. A fault needs 3
            // bad readings in a row on the same motor: one corrupted reply once read 62 °C on
            // an elbow that was at 35 °C (2026-11-24). Implausible values are ignored.
            if n % 25 == 0 {
                let k = rr % pwm_ids.len();
                let id = pwm_ids[k];
                rr += 1;
                if let Ok(t) = self.bus.read_u8(id, servo::ADDR_PRESENT_TEMPERATURE) {
                    if t <= 100 {
                        hot[k] = if t > s.max_temp_c { hot[k] + 1 } else { 0 };
                        if hot[k] >= 3 {
                            bail!("{tag} motor {id} at {t} °C (3 readings in a row)");
                        }
                    }
                }
                if let Ok(v) = self.bus.read_u8(id, servo::ADDR_PRESENT_VOLTAGE) {
                    let v = v as f64 / 10.0;
                    if (3.0..=20.0).contains(&v) {
                        low_v[k] = if v < s.min_voltage_v { low_v[k] + 1 } else { 0 };
                        if low_v[k] >= 3 {
                            bail!("{tag} motor {id} supply {v:.1} V (3 readings in a row)");
                        }
                    }
                }
            }

            let cycle_ms = cycle_start.elapsed().as_secs_f64() * 1e3;
            worst_ms = worst_ms.max(cycle_ms);
            if let Some(w) = log.as_mut() {
                let mut line = format!("{:.4},{}", unix_s(), arm.name);
                for (i, j) in arm.joints.iter().enumerate() {
                    let p = active.iter().position(|&a| a == i).map_or(0, |k| pwm_out[k]);
                    line += &format!(
                        ",{:.2},{:.3},{:.4},{},{}",
                        j.cal.raw_to_deg(st[i].pos),
                        v_f[i],
                        tau[i],
                        p,
                        st[i].load
                    );
                }
                let g = &st[arm.joints.len()];
                line += &format!(",{:.1},{},{:.3}{imp_log}\n", arm.gripper_cal.raw_to_pct(g.pos), g.load, cycle_ms);
                w.write_all(line.as_bytes())?;
            }

            n += 1;
            if opts.verbose && last_print.elapsed() > Duration::from_secs(2) {
                let hz = n as f64 / elapsed.max(1e-9);
                let rot = self.imp.as_ref().map_or(0.0, |i| i.last_rot_err_deg);
                let imp_s = imp_status.map_or(String::new(), |(m, ee, err)| {
                    format!(", imp {m} tool [{:.3} {:.3} {:.3}] m err {:.1} mm {rot:.1}°", ee[0], ee[1], ee[2], err * 1e3)
                });
                println!(
                    "{tag} {hz:.0} Hz, worst cycle {worst_ms:.2} ms, pwm {:?}, tau {:?}{imp_s}",
                    pwm_out,
                    tau.iter().map(|t| (t * 100.0).round() / 100.0).collect::<Vec<_>>()
                );
                worst_ms = 0.0;
                last_print = Instant::now();
            }

            next += period;
            let now = Instant::now();
            if next > now {
                std::thread::sleep(next - now);
            } else {
                next = now; // overrun: don't try to catch up
            }
        }
        if let Some(mut w) = log {
            w.flush()?;
        }
        Ok(())
    }
}

/// Open the gripper, wait for the user, close until the load says "holding".
/// Gripper stays in position mode the whole time.
pub fn grasp(bus: &mut Bus, arm: &Arm, cfg: &Config, object: &str, wait_enter: &dyn Fn(&str)) -> Result<()> {
    let pct = grasp_open(bus, arm, cfg)?;
    wait_enter(&format!("[{}] Put the {object} between the {} jaws and keep holding it, then press Enter", arm.name, arm.name));
    grasp_close(bus, arm, cfg, object, pct)
}

/// Gripper to position mode and open slowly to `open_pct`. Returns the opening reached.
pub fn grasp_open(bus: &mut Bus, arm: &Arm, cfg: &Config) -> Result<f64> {
    let g = &cfg.gripper;
    let cal = arm.gripper_cal;
    let id = g.id;
    let pos = bus.read_u16(id, servo::ADDR_PRESENT_POSITION)? as i32;
    bus.set_mode(id, servo::MODE_POSITION)?;
    bus.write_u16(id, servo::ADDR_GOAL_POSITION, pos as u16)?;
    bus.write_u16(id, servo::ADDR_TORQUE_LIMIT, 500)?;
    bus.set_torque(id, true)?;
    let mut pct = cal.raw_to_pct(pos);
    while (pct - g.open_pct).abs() > 0.5 {
        pct += (g.open_pct - pct).clamp(-1.0, 1.0);
        bus.write_u16(id, servo::ADDR_GOAL_POSITION, cal.pct_to_raw(pct) as u16)?;
        std::thread::sleep(Duration::from_millis(20));
    }
    Ok(pct)
}

/// Close from `pct` until the jaws meet the object; squeeze capped by hold_torque_limit.
/// Contact = the gripper has closed at least 5 % (the start-of-motion load spike is ignored)
/// and either the load reaches grasp_load or the jaw lags its goal by more than 4 % (a soft
/// object such as foam stops the jaw before the load builds up).
pub fn grasp_close(bus: &mut Bus, arm: &Arm, cfg: &Config, object: &str, mut pct: f64) -> Result<()> {
    let g = &cfg.gripper;
    let cal = arm.gripper_cal;
    let id = g.id;
    let tag = format!("[{}]", arm.name);
    let start = pct;
    bus.write_u16(id, servo::ADDR_TORQUE_LIMIT, g.hold_torque_limit.max(300))?;
    loop {
        pct -= g.close_step_pct;
        bus.write_u16(id, servo::ADDR_GOAL_POSITION, cal.pct_to_raw(pct.max(0.0)) as u16)?;
        std::thread::sleep(Duration::from_millis(20));
        let st = bus.read_state(&[id])?;
        let actual = cal.raw_to_pct(st[0].pos);
        let travelled = start - actual;
        let lag = actual - pct.max(0.0);
        if travelled >= 5.0 && (st[0].load.abs() >= g.grasp_load || lag > 4.0) {
            // Hold just inside the contact point, squeeze capped by the torque limit.
            let hold = actual - 1.5;
            bus.write_u16(id, servo::ADDR_GOAL_POSITION, cal.pct_to_raw(hold) as u16)?;
            bus.write_u16(id, servo::ADDR_TORQUE_LIMIT, g.hold_torque_limit)?;
            let why = if st[0].load.abs() >= g.grasp_load { "load" } else { "jaw stopped" };
            println!("{tag} grasped at {actual:.1} % ({why}: load {}, lag {lag:.1} %)", st[0].load);
            return Ok(());
        }
        if pct <= -3.0 {
            bus.write_u16(id, servo::ADDR_TORQUE_LIMIT, g.hold_torque_limit)?;
            bail!("{tag} gripper closed fully without feeling the {object} (at {actual:.1} %, load {})", st[0].load);
        }
    }
}

/// Gripper tilt from level (rad): (forward axis elevation, sideways axis elevation).
/// Gripper frame: local -y points forward along the jaws, local z sideways (jaw hinge),
/// local +x is the normal of a board clamped between the jaws.
pub fn gripper_tilt(model: &ArmModel, q: &[f64], link: &str) -> Result<(f64, f64)> {
    let r = model.frame_pose(q, link)?.rotation;
    let f = r * nalgebra::Vector3::new(0.0, -1.0, 0.0);
    let l = r * nalgebra::Vector3::new(0.0, 0.0, 1.0);
    Ok((f.z.clamp(-1.0, 1.0).asin(), l.z.clamp(-1.0, 1.0).asin()))
}

/// Level the gripper in position mode (stiff servo loop, ~0.1 deg) by correcting wrist_flex
/// and wrist_roll from the model: a Newton step on (pitch, roll) with the 2x2 Jacobian from
/// finite differences, repeated until both are under 0.2 deg. The compliant hold can't make
/// small orientation corrections (friction at three joints), so this runs before it.
pub fn level_wrists(bus: &mut Bus, arm: &Arm, cfg: &Config, model: &ArmModel) -> Result<()> {
    let link = format!("{}{}", cfg.impedance.as_ref().map_or("Fixed_Jaw_tip", |c| c.ee_link.as_str()), arm.cfg.urdf_suffix);
    let wf = arm.joints.iter().position(|j| j.name == "wrist_flex").ok_or_else(|| anyhow::anyhow!("no wrist_flex"))?;
    let wr = arm.joints.iter().position(|j| j.name == "wrist_roll").ok_or_else(|| anyhow::anyhow!("no wrist_roll"))?;
    let all_ids: Vec<u8> = arm.ids().into_iter().chain([cfg.gripper.id]).collect();
    let tag = format!("[{}]", arm.name);
    for iter in 0..6 {
        let st = bus.read_state(&all_ids)?;
        let q = urdf_q(arm, cfg, &st);
        let (p0, r0) = gripper_tilt(model, &q, &link)?;
        println!("{tag} level {iter}: gripper pitch {:+.2}°, roll {:+.2}°", p0.to_degrees(), r0.to_degrees());
        if p0.abs().to_degrees() < 0.2 && r0.abs().to_degrees() < 0.2 {
            return Ok(());
        }
        let h = 1e-4;
        let mut jac = [[0.0; 2]; 2];
        for (c, k) in [wf, wr].into_iter().enumerate() {
            let mut qh = q.clone();
            qh[k] += h;
            let (p1, r1) = gripper_tilt(model, &qh, &link)?;
            jac[0][c] = (p1 - p0) / h;
            jac[1][c] = (r1 - r0) / h;
        }
        let det = jac[0][0] * jac[1][1] - jac[0][1] * jac[1][0];
        if det.abs() < 1e-6 {
            bail!("{tag} cannot level the gripper from this pose (wrist singular)");
        }
        // Solve jac * dq = -(p0, r0), URDF radians; clamp each step to 10 degrees.
        let lim = 10f64.to_radians();
        let dq_f = (-(jac[1][1] * p0 - jac[0][1] * r0) / det).clamp(-lim, lim);
        let dq_r = (-(-jac[1][0] * p0 + jac[0][0] * r0) / det).clamp(-lim, lim);
        for (k, dq) in [(wf, dq_f), (wr, dq_r)] {
            let j = &arm.joints[k];
            // Add the correction to the GOAL, not the present position: a loaded servo stops a
            // little short of its goal, and stepping from the present position never closes that.
            let goal_now = bus.read_u16(j.id, servo::ADDR_GOAL_POSITION)? as i32;
            let target_deg = j.cal.raw_to_deg(goal_now) + (dq / j.sign).to_degrees();
            let (lo, hi) = j.cal.deg_range();
            let m = cfg.safety.limit_margin_deg;
            if target_deg < lo + m || target_deg > hi - m {
                bail!("{tag} levelling needs {} at {target_deg:.1}°, outside {:.1}..{:.1}", j.name, lo + m, hi - m);
            }
            // Small move, a few steps, position mode (the joint is already holding).
            let start = goal_now as f64;
            let goal = j.cal.mid() + target_deg * 4095.0 / 360.0;
            for s in 1..=10 {
                let g = start + (goal - start) * s as f64 / 10.0;
                bus.write_u16(j.id, servo::ADDR_GOAL_POSITION, g.round() as u16)?;
                std::thread::sleep(Duration::from_millis(30));
            }
        }
        std::thread::sleep(Duration::from_millis(400));
    }
    let st = bus.read_state(&all_ids)?;
    let (p, r) = gripper_tilt(model, &urdf_q(arm, cfg, &st), &link)?;
    println!("{tag} level: stopped at pitch {:+.2}°, roll {:+.2}° after 6 steps", p.to_degrees(), r.to_degrees());
    Ok(())
}

/// Move the gripper slowly to `pct` of its calibrated range (position mode), e.g. to let go of a ball.
pub fn gripper_to(bus: &mut Bus, arm: &Arm, cfg: &Config, pct: f64) -> Result<()> {
    let id = cfg.gripper.id;
    let cal = arm.gripper_cal;
    let pos = bus.read_u16(id, servo::ADDR_PRESENT_POSITION)? as i32;
    bus.write_u16(id, servo::ADDR_GOAL_POSITION, pos as u16)?;
    bus.set_mode(id, servo::MODE_POSITION)?;
    bus.write_u16(id, servo::ADDR_TORQUE_LIMIT, 500)?;
    bus.set_torque(id, true)?;
    let mut p = cal.raw_to_pct(pos);
    while (p - pct).abs() > 0.5 {
        p += (pct - p).clamp(-1.0, 1.0);
        bus.write_u16(id, servo::ADDR_GOAL_POSITION, cal.pct_to_raw(p) as u16)?;
        std::thread::sleep(Duration::from_millis(20));
    }
    Ok(())
}

/// Result of one identification sweep on one joint. Forces are in "raw-positive" PWM
/// units: f > 0 pushes the raw position up (the motor PWM sent is pwm_sign * f).
#[derive(Debug)]
#[allow(dead_code)] // fields shown via Debug
pub struct Ident {
    pub deg: f64,
    /// Model holding torque in the motor's direction (N·m).
    pub tau_model: f64,
    /// Settling drift after entering PWM 0 (steps), not counted as motion.
    pub settle_steps: i32,
    /// Break-away force against gravity and with gravity.
    pub f_against: i32,
    pub f_with: i32,
}

impl Ident {
    pub fn hold(&self) -> f64 {
        (self.f_against + self.f_with) as f64 / 2.0
    }
    pub fn friction(&self) -> f64 {
        (self.f_against - self.f_with).abs() as f64 / 2.0
    }
}

const MOVE_STEPS: i32 = 8; // 0.7°: counts as "started moving"
const MAX_SLIP_STEPS: i32 = 40; // 3.5°: abort

/// One continuous sweep at the present pose: ramp the force against gravity until the
/// joint moves, then ramp back until it moves with gravity. Other joints hold position.
/// Uses the joint's configured pwm_sign; an unloaded joint moving the wrong way is
/// reported as a pwm_sign error.
pub fn identify(bus: &mut Bus, arm: &Arm, cfg: &Config, model: &ArmModel, jname: &str) -> Result<Ident> {
    let ji = arm.joints.iter().position(|j| j.name == jname).ok_or_else(|| anyhow::anyhow!("no joint {jname}"))?;
    let j = &arm.joints[ji];
    let ids = arm.ids();
    bus.hold_here(&ids, cfg.safety.hold_torque_limit)?;
    std::thread::sleep(Duration::from_millis(300));
    let all_ids: Vec<u8> = ids.iter().copied().chain([cfg.gripper.id]).collect();
    let st = bus.read_state(&all_ids)?;
    let q = urdf_q(arm, cfg, &st);
    let tau = motor_gravity(arm, model, &q)[ji];
    let deg = j.cal.raw_to_deg(st[ji].pos);
    let max = j.max_pwm.round() as i32;
    let dir0: i32 = if tau >= 0.0 { 1 } else { -1 }; // raw direction that lifts against gravity
    let pos = |bus: &mut Bus| -> Result<i32> { Ok(bus.read_state(&[j.id])?[0].pos) };
    let send = |bus: &mut Bus, f: i32| bus.write_pwm(&[j.id], &[(j.pwm_sign * f as f64).round() as i32]);

    // Loaded joints slip at PWM 0 (the shoulder carries the whole arm), so start at 60 %
    // of the model's holding force; the sweep then finds the real break-away both ways.
    let f_start = dir0 * ((0.6 * tau.abs() * j.pwm_per_nm).round() as i32).min(max / 2);
    let p_hold = pos(bus)?;
    bus.enter_pwm(&[j.id], &[max as u16])?;
    send(bus, f_start)?;
    std::thread::sleep(Duration::from_millis(120));
    let p0 = pos(bus)?;
    let settle_steps = p0 - p_hold;
    println!("{jname}: moved {settle_steps:+} steps in 120 ms at start force {f_start:+}; model tau {tau:+.3} N·m; lifting direction raw {dir0:+}");
    if settle_steps.abs() > MAX_SLIP_STEPS {
        bus.hold_here(&[j.id], cfg.safety.hold_torque_limit)?;
        bail!("{jname} fell {settle_steps:+} steps at PWM 0 (gravity beats friction here); identify in a pose with less load");
    }

    let result = (|| -> Result<(i32, i32)> {
        // Phase 1: f from 0 toward dir0 until the joint moves MOVE_STEPS in dir0.
        let mut f: i32 = f_start;
        let mut low: i32 = 0; // most negative displacement along dir0 (slip with gravity)
        let f_against = loop {
            if f.abs() > max {
                bail!("{jname} did not lift within max_pwm {max}; raise max_pwm");
            }
            send(bus, f)?;
            std::thread::sleep(Duration::from_millis(25));
            let d = dir0 * (pos(bus)? - p0);
            low = low.min(d);
            if d - low >= MOVE_STEPS && f.abs() >= 6 {
                break f;
            }
            if -low > MAX_SLIP_STEPS {
                bail!(
                    "{jname} moved {} steps opposite to the command (pwm_sign is probably {} for this joint, or gravity beats max_pwm)",
                    -low,
                    -j.pwm_sign
                );
            }
            f += 2 * dir0;
        };
        // Phase 2: stop the joint (sliding friction < static, it would keep rising), then
        // from rest ramp from 90 % of f_against toward -dir0 until it moves with gravity.
        send(bus, f_start)?;
        bus.hold_here(&[j.id], cfg.safety.hold_torque_limit)?;
        std::thread::sleep(Duration::from_millis(250));
        let p1 = pos(bus)?;
        bus.enter_pwm(&[j.id], &[max as u16])?;
        let mut f = f_against - dir0 * (f_against.abs() / 10).max(6);
        let mut high = i32::MIN;
        loop {
            if f.abs() > max {
                bail!("{jname} did not move back within max_pwm {max}");
            }
            send(bus, f)?;
            std::thread::sleep(Duration::from_millis(25));
            let d = dir0 * (pos(bus)? - p1);
            high = high.max(d);
            if high - d >= MOVE_STEPS {
                return Ok((f_against, f));
            }
            if d > MAX_SLIP_STEPS {
                bail!("{jname} kept rising past {MAX_SLIP_STEPS} steps");
            }
            f -= 2 * dir0;
        }
    })();
    let _ = send(bus, 0);
    bus.hold_here(&[j.id], cfg.safety.hold_torque_limit)?;
    let (f_against, f_with) = result?;
    Ok(Ident { deg, tau_model: tau, settle_steps, f_against, f_with })
}

/// Move the 5 arm joints in position mode to `target_deg` (LeRobot degrees), slowly.
/// Leaves them holding the pose in position mode. The gripper is not touched.
pub fn move_to_pose(bus: &mut Bus, arm: &Arm, cfg: &Config, target_deg: &[f64], speed_dps: f64) -> Result<()> {
    let ids = arm.ids();
    if target_deg.len() != ids.len() {
        bail!("pose needs {} joint angles, got {}", ids.len(), target_deg.len());
    }
    for (j, &t) in arm.joints.iter().zip(target_deg) {
        let (lo, hi) = j.cal.deg_range();
        let m = cfg.safety.limit_margin_deg;
        if t < lo + m || t > hi - m {
            bail!("[{}] pose {} = {t:.1}° is outside {:.1}..{:.1}", arm.name, j.name, lo + m, hi - m);
        }
    }
    bus.hold_here(&ids, 600)?; // moderate torque while moving
    let st = bus.read_state(&ids)?;
    let start: Vec<f64> = arm.joints.iter().zip(&st).map(|(j, s)| j.cal.raw_to_deg(s.pos)).collect();
    let dist = start.iter().zip(target_deg).map(|(a, b)| (a - b).abs()).fold(0.0, f64::max);
    let steps = ((dist / speed_dps) / 0.02).ceil().max(1.0) as usize;
    println!("[{}] moving to pose ({dist:.0}° max travel, {:.1} s)", arm.name, steps as f64 * 0.02);
    for k in 1..=steps {
        let a = k as f64 / steps as f64;
        let a = a * a * (3.0 - 2.0 * a); // smoothstep
        for ((j, s), t) in arm.joints.iter().zip(&start).zip(target_deg) {
            let deg = s + (t - s) * a;
            let raw = (j.cal.mid() + deg * 4095.0 / 360.0).round() as u16;
            bus.write_u16(j.id, servo::ADDR_GOAL_POSITION, raw)?;
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    std::thread::sleep(Duration::from_millis(400));
    for &id in &ids {
        bus.write_u16(id, servo::ADDR_TORQUE_LIMIT, cfg.safety.hold_torque_limit)?;
    }
    Ok(())
}
