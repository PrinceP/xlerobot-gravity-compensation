//! gravcomp: gravity compensation for the XLeRobot SO-101 arms (Feetech STS3215).
//!
//! Plain-Rust take on NVIDIA's "Make a robot arm feel weightless with Holoscan" (reBot B601-DM):
//! read joints -> gravity torque from the URDF -> torque command, at a fixed rate, with a
//! watchdog. The STS3215 has no torque mode, so the torque command is PWM duty (see servo.rs).

mod config;
mod control;
mod model;
mod servo;

use std::io::BufRead;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use anyhow::{Context, Result, bail};
use clap::{Parser, Subcommand};

use config::{Arm, Config, expand};
use control::{Loop, RunOpts};
use model::{ArmModel, ExtraMass};
use servo::Bus;

#[derive(Parser)]
#[command(version, about = "Gravity compensation for XLeRobot arms")]
struct Cli {
    /// Config file.
    #[arg(long, default_value = "config/xlerobot.toml", global = true)]
    config: PathBuf,
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Subcommand)]
enum Cmd {
    /// Read-only: bus check, joint angles, model gravity torque vs measured load. Moves nothing.
    Probe {
        #[arg(long, default_value = "both")]
        arm: String,
    },
    /// Offline: model gravity torque at a URDF pose (rad, 6 values incl. jaw). Prints JSON.
    Gravity {
        #[arg(long, default_value = "left")]
        arm: String,
        #[arg(long, value_delimiter = ',', allow_hyphen_values = true)]
        q: Vec<f64>,
    },
    /// Move arm(s) slowly to a named pose in position mode and hold it.
    Pose {
        #[arg(long, default_value = "both")]
        arm: String,
        #[arg(long, default_value = "extended")]
        name: String,
    },
    /// Measure break-away PWM both ways for one joint at the present pose; suggests gains.
    Identify {
        #[arg(long)]
        arm: String,
        #[arg(long)]
        joint: String,
    },
    /// Run gravity compensation. Ctrl+C holds the arm where it is.
    Run {
        #[arg(long, default_value = "both")]
        arm: String,
        /// Overall gain on the gravity term (start low, e.g. 0.5).
        #[arg(long, default_value_t = 0.5)]
        scale: f64,
        /// Only these joints (comma list); the rest hold position.
        #[arg(long, value_delimiter = ',')]
        joints: Vec<String>,
        /// Move to this named pose first (e.g. "extended").
        #[arg(long)]
        pose: Option<String>,
        /// Open grippers, wait for a ping pong ball each, close on it, then run.
        #[arg(long)]
        grasp: bool,
        /// Stop after this many seconds.
        #[arg(long)]
        duration: Option<f64>,
        /// CSV log; "{arm}" is replaced by the arm name.
        #[arg(long)]
        log: Option<String>,
        #[arg(long, short)]
        verbose: bool,
    },
    /// Put every arm motor back in position mode, holding where it is.
    Hold {
        #[arg(long, default_value = "both")]
        arm: String,
    },
    /// Torque OFF on all arm motors (the arm falls: support it first). Needs --yes.
    Release {
        #[arg(long, default_value = "both")]
        arm: String,
        #[arg(long)]
        yes: bool,
    },
}

fn arm_names(sel: &str) -> Vec<String> {
    match sel {
        "both" => vec!["left".into(), "right".into()],
        s => vec![s.to_string()],
    }
}

fn resolve(cfg_path: &Path, p: &str) -> PathBuf {
    let e = expand(p);
    if e.is_absolute() || e.exists() {
        return e;
    }
    cfg_path.parent().and_then(Path::parent).map_or(e.clone(), |root| root.join(&e))
}

fn load_model(cli_cfg: &Path, cfg: &Config, arm: &Arm) -> Result<ArmModel> {
    let extra: Vec<ExtraMass> = cfg
        .extra_mass
        .iter()
        .map(|e| ExtraMass { link: format!("{}{}", e.link, arm.cfg.urdf_suffix), ..e.clone() })
        .collect();
    ArmModel::from_urdf(
        &resolve(cli_cfg, &cfg.urdf),
        &cfg.world_link,
        &arm.cfg.urdf_root,
        &arm.urdf_joint_names(&cfg.gripper),
        &extra,
    )
}

/// Open the arm's bus and make sure it is the right one (ttyACM numbers can swap).
fn open_bus(cfg: &Config, arm: &Arm) -> Result<Bus> {
    let mut bus = Bus::open(&arm.cfg.port)?;
    let ids: Vec<u8> = arm.ids().into_iter().chain([cfg.gripper.id]).collect();
    let missing: Vec<u8> = ids.iter().copied().filter(|&id| !bus.ping(id)).collect();
    if !missing.is_empty() {
        bail!("[{}] {}: motors {missing:?} do not answer", arm.name, arm.cfg.port);
    }
    let has9 = bus.ping(9);
    let ok = if arm.cfg.bus_marker_id == 0 { !has9 } else { bus.ping(arm.cfg.bus_marker_id) };
    if !ok {
        bail!(
            "[{}] {} looks like the other arm's bus (motor 9 {}). Swap the ports in the config.",
            arm.name,
            arm.cfg.port,
            if has9 { "present" } else { "absent" }
        );
    }
    Ok(bus)
}

fn wait_enter(msg: &str) {
    println!("{msg}");
    let mut s = String::new();
    let _ = std::io::stdin().lock().read_line(&mut s);
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    let cfg = Config::load(&cli.config)?;

    match &cli.cmd {
        Cmd::Gravity { arm, q } => {
            let a = cfg.arm_with(arm, false)?;
            let m = load_model(&cli.config, &cfg, &a)?;
            if q.len() != m.joint_names.len() {
                bail!("--q needs {} values ({:?})", m.joint_names.len(), m.joint_names);
            }
            let g = m.gravity(q);
            println!("{}", serde_json::json!({"joints": m.joint_names, "q": q, "tau": g, "mass": m.total_mass()}));
        }

        Cmd::Probe { arm } => {
            for name in arm_names(arm) {
                let a = cfg.arm(&name)?;
                let m = load_model(&cli.config, &cfg, &a)?;
                let mut bus = open_bus(&cfg, &a)?;
                let ids: Vec<u8> = a.ids().into_iter().chain([cfg.gripper.id]).collect();
                let t = std::time::Instant::now();
                let st = bus.read_state(&ids)?;
                let read_ms = t.elapsed().as_secs_f64() * 1e3;
                let q = control::urdf_q(&a, &cfg, &st);
                let tau = control::motor_gravity(&a, &m, &q);
                println!("== {name} arm on {} (arm mass in model {:.3} kg, state read {read_ms:.2} ms)", a.cfg.port, m.total_mass());
                println!("{:14} {:>5} {:>8} {:>8} {:>9} {:>6} {:>5} {:>6} {:>5}", "joint", "raw", "deg", "urdf", "tau_Nm", "load", "mode", "torque", "°C");
                for (i, j) in a.joints.iter().enumerate() {
                    let mode = bus.read_u8(j.id, servo::ADDR_OPERATING_MODE)?;
                    let te = bus.read_u8(j.id, servo::ADDR_TORQUE_ENABLE)?;
                    let tc = bus.read_u8(j.id, servo::ADDR_PRESENT_TEMPERATURE)?;
                    println!(
                        "{:14} {:>5} {:>8.1} {:>8.3} {:>9.3} {:>6} {:>5} {:>6} {:>5}",
                        j.name, st[i].pos, j.cal.raw_to_deg(st[i].pos), q[i], tau[i], st[i].load, mode, te, tc
                    );
                }
                let g = st[a.joints.len()];
                println!("{:14} {:>5} {:>7.1}% {:>8.3}", "gripper", g.pos, a.gripper_cal.raw_to_pct(g.pos), q[a.joints.len()]);
                let v = bus.read_u8(a.ids()[0], servo::ADDR_PRESENT_VOLTAGE)? as f64 / 10.0;
                println!("supply {v:.1} V");
            }
        }

        Cmd::Pose { arm, name } => {
            let (pose, speed) = cfg.pose(name)?;
            for n in arm_names(arm) {
                let a = cfg.arm(&n)?;
                let mut bus = open_bus(&cfg, &a)?;
                control::move_to_pose(&mut bus, &a, &cfg, &pose, speed)?;
            }
            println!("Holding pose '{name}' (position mode).");
        }

        Cmd::Identify { arm, joint } => {
            let a = cfg.arm(arm)?;
            let m = load_model(&cli.config, &cfg, &a)?;
            let mut bus = open_bus(&cfg, &a)?;
            let r = control::identify(&mut bus, &a, &cfg, &m, joint)?;
            println!("{r:?}");
            println!(
                "{joint} at {:.1}°: break-away against gravity {:+}, with gravity {:+} (PWM, raw-positive)",
                r.deg, r.f_against, r.f_with
            );
            println!("  friction_pwm ≈ {:.0}   gravity hold ≈ {:+.0} PWM", r.friction(), r.hold());
            if r.tau_model.abs() > 0.05 {
                let k = r.hold() / r.tau_model;
                println!("  pwm_per_nm ≈ {k:.0}   (model tau {:+.3} N·m)", r.tau_model);
                if k < 0.0 {
                    println!("  NEGATIVE: the model and the motor disagree on direction. Check sign/offset for {joint} before `run`.");
                }
            } else {
                println!("  model torque ~0 here: no pwm_per_nm from this pose (friction and pwm_sign are still valid)");
            }
            println!("[{arm}] joint held in position mode.");
        }

        Cmd::Hold { arm } => {
            for n in arm_names(arm) {
                let a = cfg.arm(&n)?;
                let mut bus = open_bus(&cfg, &a)?;
                bus.hold_here(&a.ids(), cfg.safety.hold_torque_limit)?;
                println!("[{n}] holding");
            }
        }

        Cmd::Release { arm, yes } => {
            if !yes {
                bail!("release turns torque off and the arm falls. Support it, then pass --yes.");
            }
            for n in arm_names(arm) {
                let a = cfg.arm(&n)?;
                let mut bus = open_bus(&cfg, &a)?;
                bus.release(&a.ids())?;
                println!("[{n}] torque off");
            }
        }

        Cmd::Run { arm, scale, joints, pose, grasp, duration, log, verbose } => {
            if !(0.0..=1.5).contains(scale) {
                bail!("--scale must be in 0..1.5");
            }
            let stop = Arc::new(AtomicBool::new(false));
            {
                let stop = stop.clone();
                ctrlc::set_handler(move || {
                    if stop.swap(true, Ordering::SeqCst) {
                        eprintln!("second Ctrl+C: exiting without hold");
                        std::process::exit(130);
                    }
                    eprintln!("stopping: switching to hold...");
                })
                .context("Ctrl+C handler")?;
            }

            let mut arms = Vec::new();
            for n in arm_names(arm) {
                let mut a = cfg.arm(&n)?;
                if !joints.is_empty() {
                    for j in &mut a.joints {
                        j.enabled = j.enabled && joints.contains(&j.name);
                    }
                }
                let m = load_model(&cli.config, &cfg, &a)?;
                let bus = open_bus(&cfg, &a)?;
                arms.push((a, m, bus));
            }
            if let Some(p) = pose {
                let (q, speed) = cfg.pose(p)?;
                for (a, _, bus) in &mut arms {
                    control::move_to_pose(bus, a, &cfg, &q, speed)?;
                }
            }
            if *grasp {
                for (a, _, bus) in &mut arms {
                    control::grasp(bus, a, &cfg, &wait_enter)?;
                }
            }
            if stop.load(Ordering::SeqCst) {
                bail!("stopped before start");
            }

            let failed = std::thread::scope(|sc| {
                let handles: Vec<_> = arms
                    .into_iter()
                    .map(|(a, m, bus)| {
                        let stop = stop.clone();
                        let cfg = &cfg;
                        let log = log.as_ref().map(|l| l.replace("{arm}", &a.name));
                        sc.spawn(move || {
                            let opts = RunOpts { scale: *scale, duration: *duration, log, verbose: *verbose };
                            let mut lp = Loop { arm: &a, cfg, model: &m, bus };
                            let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| lp.run(&opts, &stop)));
                            stop.store(true, Ordering::SeqCst); // one arm stops -> both stop
                            let held = lp.bus.hold_here(&a.ids(), cfg.safety.hold_torque_limit);
                            let res = match r {
                                Ok(r) => r,
                                Err(_) => Err(anyhow::anyhow!("[{}] control loop panicked", a.name)),
                            };
                            if let Err(e) = &res {
                                eprintln!("[{}] fault: {e:#}", a.name);
                            }
                            match &held {
                                Err(e) => eprintln!("[{}] HOLD FAILED: {e:#}. Run `make hold` or cut power.", a.name),
                                Ok(()) => println!("[{}] stopped, holding position", a.name),
                            }
                            res.is_err() || held.is_err()
                        })
                    })
                    .collect();
                handles.into_iter().map(|h| h.join().unwrap_or(true)).any(|f| f)
            });
            if failed {
                std::process::exit(1);
            }
        }
    }
    Ok(())
}
