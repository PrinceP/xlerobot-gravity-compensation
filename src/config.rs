//! Configuration: `config/xlerobot.toml` plus the LeRobot calibration JSON.

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail};
use serde::Deserialize;

use crate::model::ExtraMass;

#[derive(Debug, Clone, Deserialize)]
pub struct Config {
    pub calibration: String,
    pub urdf: String,
    pub world_link: String,
    pub rate_hz: f64,
    pub ramp_s: f64,
    /// Gravity term starts at this fraction at t = 0 (0 = from nothing, for untested gains).
    #[serde(default)]
    pub ramp_from: f64,
    pub safety: Safety,
    pub gripper: Gripper,
    pub joints: Vec<JointCfg>,
    pub poses: HashMap<String, toml::Value>,
    #[serde(default)]
    pub extra_mass: Vec<ExtraMass>,
    pub arms: HashMap<String, ArmCfg>,
    pub impedance: Option<crate::impedance::ImpCfg>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct Safety {
    pub max_speed_radps: f64,
    pub limit_margin_deg: f64,
    pub wall_pwm_per_deg: f64,
    pub max_temp_c: u8,
    pub min_voltage_v: f64,
    pub max_comm_errors: u32,
    pub hold_torque_limit: u16,
}

#[derive(Debug, Clone, Deserialize)]
pub struct Gripper {
    pub id: u8,
    pub urdf: String,
    /// URDF jaw angle (rad) at 100 % of the calibrated gripper range.
    pub span_rad: f64,
    pub open_pct: f64,
    pub close_step_pct: f64,
    /// Stop closing when |Present_Load| (0.1 % units) exceeds this.
    pub grasp_load: i32,
    /// Torque_Limit while holding the ball (0..1000): caps the squeeze.
    pub hold_torque_limit: u16,
}

#[derive(Debug, Clone, Deserialize)]
pub struct JointCfg {
    pub name: String,
    pub id: u8,
    pub urdf: String,
    /// q_urdf = sign * q_lerobot_rad + offset_rad
    pub sign: f64,
    pub offset_rad: f64,
    #[serde(flatten)]
    pub gains: Gains,
}

#[derive(Debug, Clone, Deserialize, Default)]
pub struct Gains {
    pub enabled: Option<bool>,
    /// PWM units (of 1000) per N·m at the joint.
    pub pwm_per_nm: Option<f64>,
    /// +1 if positive PWM moves the raw position up.
    pub pwm_sign: Option<f64>,
    pub gravity_scale: Option<f64>,
    /// Coulomb friction feedforward, PWM units, applied as tanh(v / friction_vel).
    pub friction_pwm: Option<f64>,
    pub friction_vel: Option<f64>,
    /// PWM per rad/s. Positive assists motion (cancels back-EMF), negative damps.
    pub viscous_pwm: Option<f64>,
    pub max_pwm: Option<f64>,
    /// Measured static (break-away) friction, PWM. Sizes the impedance at-rest assist.
    pub stiction_pwm: Option<f64>,
}

impl Gains {
    fn or(&self, o: &Gains) -> Gains {
        Gains {
            enabled: o.enabled.or(self.enabled),
            pwm_per_nm: o.pwm_per_nm.or(self.pwm_per_nm),
            pwm_sign: o.pwm_sign.or(self.pwm_sign),
            gravity_scale: o.gravity_scale.or(self.gravity_scale),
            friction_pwm: o.friction_pwm.or(self.friction_pwm),
            friction_vel: o.friction_vel.or(self.friction_vel),
            viscous_pwm: o.viscous_pwm.or(self.viscous_pwm),
            max_pwm: o.max_pwm.or(self.max_pwm),
            stiction_pwm: o.stiction_pwm.or(self.stiction_pwm),
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct ArmCfg {
    pub port: String,
    pub urdf_root: String,
    pub urdf_suffix: String,
    pub calib_prefix: String,
    /// Motor id that exists only on this arm's bus, to catch swapped ttyACM ports.
    pub bus_marker_id: u8,
    /// UDP port for impedance goals (`run --listen`).
    pub goal_port: Option<u16>,
    #[serde(default)]
    pub overrides: HashMap<String, Gains>,
}

/// Fully resolved joint for one arm.
#[derive(Debug, Clone)]
pub struct Joint {
    pub name: String,
    pub id: u8,
    pub urdf: String,
    pub sign: f64,
    pub offset_rad: f64,
    pub enabled: bool,
    pub pwm_per_nm: f64,
    pub pwm_sign: f64,
    pub gravity_scale: f64,
    pub friction_pwm: f64,
    pub friction_vel: f64,
    pub viscous_pwm: f64,
    pub max_pwm: f64,
    pub stiction_pwm: f64,
    pub cal: Calib,
}

#[derive(Debug, Clone, Copy, Deserialize)]
pub struct Calib {
    pub range_min: i32,
    pub range_max: i32,
}

impl Calib {
    pub fn mid(&self) -> f64 {
        (self.range_min + self.range_max) as f64 / 2.0
    }
    /// LeRobot DEGREES normalisation: (raw - mid) * 360 / 4095.
    pub fn raw_to_deg(&self, raw: i32) -> f64 {
        (raw as f64 - self.mid()) * 360.0 / 4095.0
    }
    pub fn deg_range(&self) -> (f64, f64) {
        (self.raw_to_deg(self.range_min), self.raw_to_deg(self.range_max))
    }
    pub fn raw_to_pct(&self, raw: i32) -> f64 {
        (raw - self.range_min) as f64 * 100.0 / (self.range_max - self.range_min) as f64
    }
    pub fn pct_to_raw(&self, pct: f64) -> i32 {
        self.range_min + (pct.clamp(0.0, 100.0) / 100.0 * (self.range_max - self.range_min) as f64).round() as i32
    }
}

#[derive(Debug, Clone)]
pub struct Arm {
    pub name: String,
    pub cfg: ArmCfg,
    pub joints: Vec<Joint>,
    pub gripper_cal: Calib,
}

impl Arm {
    pub fn ids(&self) -> Vec<u8> {
        self.joints.iter().map(|j| j.id).collect()
    }
    pub fn urdf_joint_names(&self, gripper: &Gripper) -> Vec<String> {
        self.joints
            .iter()
            .map(|j| j.urdf.clone())
            .chain(std::iter::once(gripper.urdf.clone()))
            .map(|n| format!("{n}{}", self.cfg.urdf_suffix))
            .collect()
    }
}

pub fn expand(p: &str) -> PathBuf {
    match p.strip_prefix("~/") {
        Some(rest) => PathBuf::from(std::env::var("HOME").unwrap_or_default()).join(rest),
        None => PathBuf::from(p),
    }
}

impl Config {
    pub fn load(path: &Path) -> Result<Self> {
        let text = std::fs::read_to_string(path).with_context(|| format!("read {}", path.display()))?;
        let cfg: Config = toml::from_str(&text).with_context(|| format!("parse {}", path.display()))?;
        Ok(cfg)
    }

    /// A named pose (LeRobot degrees) and the move speed.
    pub fn pose(&self, name: &str) -> Result<(Vec<f64>, f64)> {
        let speed = self.poses.get("pose_speed_dps").and_then(|v| v.as_float()).unwrap_or(20.0);
        let v = self.poses.get(name).and_then(|v| v.as_array()).with_context(|| format!("no pose '{name}' in [poses]"))?;
        let q = v.iter().map(|x| x.as_float().or(x.as_integer().map(|i| i as f64))).collect::<Option<Vec<f64>>>()
            .with_context(|| format!("pose '{name}' must be numbers"))?;
        Ok((q, speed))
    }

    pub fn arm(&self, name: &str) -> Result<Arm> {
        self.arm_with(name, true)
    }

    /// `need_cal = false`: use a full 0..4095 range if the calibration file is missing
    /// (offline model commands only).
    pub fn arm_with(&self, name: &str, need_cal: bool) -> Result<Arm> {
        let Some(acfg) = self.arms.get(name) else {
            bail!("no arm '{name}' in config (have: {:?})", self.arms.keys().collect::<Vec<_>>());
        };
        let cal_path = expand(&self.calibration);
        let cal: HashMap<String, Calib> = match std::fs::read_to_string(&cal_path) {
            Ok(t) => serde_json::from_str(&t)?,
            Err(_) if !need_cal => HashMap::new(),
            Err(e) => return Err(e).with_context(|| format!("read LeRobot calibration {}", cal_path.display())),
        };
        let get = |motor: &str| -> Result<Calib> {
            if !need_cal && cal.is_empty() {
                return Ok(Calib { range_min: 0, range_max: 4095 });
            }
            let key = format!("{}{motor}", acfg.calib_prefix);
            cal.get(&key).copied().with_context(|| format!("{key} missing from {}", cal_path.display()))
        };
        let mut joints = Vec::new();
        for j in &self.joints {
            let g = j.gains.or(acfg.overrides.get(&j.name).unwrap_or(&Gains::default()));
            joints.push(Joint {
                name: j.name.clone(),
                id: j.id,
                urdf: j.urdf.clone(),
                sign: j.sign,
                offset_rad: j.offset_rad,
                enabled: g.enabled.unwrap_or(true),
                pwm_per_nm: g.pwm_per_nm.context("pwm_per_nm")?,
                pwm_sign: g.pwm_sign.unwrap_or(1.0),
                gravity_scale: g.gravity_scale.unwrap_or(1.0),
                friction_pwm: g.friction_pwm.unwrap_or(0.0),
                friction_vel: g.friction_vel.unwrap_or(0.2),
                viscous_pwm: g.viscous_pwm.unwrap_or(0.0),
                max_pwm: g.max_pwm.unwrap_or(400.0),
                stiction_pwm: g.stiction_pwm.or(g.friction_pwm).unwrap_or(0.0),
                cal: get(&j.name)?,
            });
        }
        Ok(Arm { name: name.to_string(), cfg: acfg.clone(), joints, gripper_cal: get("gripper")? })
    }
}
