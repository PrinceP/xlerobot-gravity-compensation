//! Impedance control on top of gravity compensation (NVIDIA tutorial's second example).
//!
//! A goal pulls the arm like a spring; you can still push it away by hand.
//!   joint goal:  tau = Kq (q_d - q) - Dq qd
//!   pose goal:   tau = J^T [ Kp (p_d - p) - Dp v ;  Ko log(R_d R^T) - Do w ] - Dq qd
//! in the arm base frame (`Base` / `Base_2`, the fixed mount), at the tool link
//! (default `Fixed_Jaw_tip`). The SO-101 has 5 joints, so orientation components it
//! cannot reach are simply not produced by J^T. Force, moment and joint torque are capped.
//!
//! Goals arrive as JSON over UDP (one port per arm), e.g. from a LeRobot policy:
//!   {"mode":"joints","q_deg":[pan,lift,elbow,wrist_flex,wrist_roll]}   LeRobot degrees
//!   {"mode":"pose","pos":[x,y,z],"quat":[w,x,y,z]}                    metres, quat optional
//!   {"mode":"hold"}   spring to the current tool pose      {"mode":"off"}   gravity only
//!   {"mode":"ping"}   no change; just (re)start the state stream to the sender
//!   optional: "ttl": seconds the goal stays valid (default goal_ttl_s; 0 = until replaced)
//!             "stiffness": 0..2, scales the spring gains
//! A goal that expires (no new message within ttl) falls back to "off": the policy
//! stopping leaves a compliant, gravity-compensated arm. The commanded goal moves toward
//! the received one at a limited speed, starting from the present state, so a jump in
//! the goal never becomes a jump in force. State is streamed back to the last sender.

use std::net::{SocketAddr, UdpSocket};
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use nalgebra::{DMatrix, DVector, Isometry3, Quaternion, UnitQuaternion, Vector3};
use serde::Deserialize;

use crate::config::Arm;
use crate::model::ArmModel;

#[derive(Debug, Clone, Deserialize)]
pub struct ImpCfg {
    pub ee_link: String,
    pub kp_lin: f64,
    pub kd_lin: f64,
    pub kp_rot: f64,
    pub kd_rot: f64,
    pub kq: f64,
    pub dq: f64,
    pub max_force: f64,
    pub max_moment: f64,
    pub max_joint_tau: f64,
    pub goal_ttl_s: f64,
    pub max_goal_speed: f64,
    pub max_goal_rot_speed: f64,
    pub max_joint_goal_speed_dps: f64,
    pub reach_m: f64,
    pub state_hz: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Mode {
    Off,
    Joints,
    Pose,
}

#[derive(Debug, Deserialize)]
struct Msg {
    mode: String,
    q_deg: Option<Vec<f64>>,
    pos: Option<[f64; 3]>,
    quat: Option<[f64; 4]>,
    ttl: Option<f64>,
    stiffness: Option<f64>,
}

pub struct Impedance {
    cfg: ImpCfg,
    sock: Option<UdpSocket>,
    peer: Option<SocketAddr>,
    last_rx: Instant,
    last_state_tx: Instant,
    mode: Mode,
    expires: Option<Instant>,
    stiffness: f64,
    // Received targets.
    q_target: Vec<f64>, // URDF rad, 5
    p_target: Vector3<f64>,
    r_target: Option<UnitQuaternion<f64>>,
    /// Start "pose" mode at the present tool pose when the targets are first needed.
    pending_hold: bool,
    // Rate-limited commanded goals (what the spring pulls toward).
    q_cmd: Vec<f64>,
    p_cmd: Vector3<f64>,
    r_cmd: UnitQuaternion<f64>,
    fresh: bool, // next cycle initialises *_cmd from the present state
    pub last_err_m: f64,
}

pub struct ImpOut {
    /// Spring/damper torque per joint, URDF frame, N·m (5 joints).
    pub tau: Vec<f64>,
    pub ee: Vector3<f64>,
    pub mode_name: &'static str,
}

impl Impedance {
    pub fn new(cfg: ImpCfg, listen: Option<&str>, start_hold: bool) -> Result<Self> {
        let sock = match listen {
            Some(addr) => {
                let s = UdpSocket::bind(addr).with_context(|| format!("bind UDP {addr}"))?;
                s.set_nonblocking(true)?;
                Some(s)
            }
            None => None,
        };
        Ok(Self {
            cfg,
            sock,
            peer: None,
            last_rx: Instant::now(),
            last_state_tx: Instant::now(),
            mode: Mode::Off,
            expires: None,
            stiffness: 1.0,
            q_target: vec![0.0; 5],
            p_target: Vector3::zeros(),
            r_target: None,
            pending_hold: start_hold,
            q_cmd: vec![0.0; 5],
            p_cmd: Vector3::zeros(),
            r_cmd: UnitQuaternion::identity(),
            fresh: true,
            last_err_m: 0.0,
        })
    }

    /// Tool pose in the arm base frame.
    fn tool(&self, model: &ArmModel, q: &[f64]) -> Result<(Isometry3<f64>, DMatrix<f64>)> {
        let (w, jw) = model.jacobian(q, &self.cfg.ee_link)?;
        let root = model.root_pose();
        let rt = root.rotation.inverse().to_rotation_matrix();
        let mut jb = DMatrix::zeros(6, 5);
        for k in 0..5 {
            let lin = rt * Vector3::new(jw[(0, k)], jw[(1, k)], jw[(2, k)]);
            let ang = rt * Vector3::new(jw[(3, k)], jw[(4, k)], jw[(5, k)]);
            for r in 0..3 {
                jb[(r, k)] = lin[r];
                jb[(r + 3, k)] = ang[r];
            }
        }
        Ok((root.inverse() * w, jb))
    }

    fn handle(&mut self, m: Msg, arm: &Arm) {
        if m.mode == "ping" {
            return; // subscribe to state only
        }
        let ttl = m.ttl.unwrap_or(self.cfg.goal_ttl_s);
        self.expires = (ttl > 0.0).then(|| Instant::now() + Duration::from_secs_f64(ttl));
        self.stiffness = m.stiffness.unwrap_or(1.0).clamp(0.0, 2.0);
        let prev = self.mode;
        match m.mode.as_str() {
            "joints" => {
                if let Some(q) = m.q_deg.filter(|q| q.len() == 5) {
                    for (i, j) in arm.joints.iter().enumerate() {
                        let (lo, hi) = j.cal.deg_range();
                        let d = q[i].clamp(lo + 8.0, hi - 8.0);
                        self.q_target[i] = j.sign * d.to_radians() + j.offset_rad;
                    }
                    self.mode = Mode::Joints;
                }
            }
            "pose" => {
                if let Some(p) = m.pos {
                    let mut v = Vector3::new(p[0], p[1], p[2]);
                    if v.norm() > self.cfg.reach_m {
                        v *= self.cfg.reach_m / v.norm();
                    }
                    self.p_target = v;
                    self.r_target = m.quat.map(|q| UnitQuaternion::from_quaternion(Quaternion::new(q[0], q[1], q[2], q[3])));
                    self.mode = Mode::Pose;
                }
            }
            "hold" => {
                self.pending_hold = true;
            }
            _ => {
                self.mode = Mode::Off;
            }
        }
        if self.mode != prev {
            self.fresh = true;
        }
    }

    /// Read pending goal messages; call once per cycle.
    pub fn poll(&mut self, arm: &Arm) {
        let Some(sock) = &self.sock else { return };
        let mut buf = [0u8; 2048];
        let mut msgs = Vec::new();
        while let Ok((n, from)) = sock.recv_from(&mut buf) {
            self.peer = Some(from);
            self.last_rx = Instant::now();
            match serde_json::from_slice::<Msg>(&buf[..n]) {
                Ok(m) => msgs.push(m),
                Err(e) => eprintln!("[{}] bad goal message: {e}", arm.name),
            }
        }
        for m in msgs {
            self.handle(m, arm);
        }
    }

    /// Spring/damper torque for this cycle. `q`: URDF angles (6, incl. jaw), `qd`: URDF
    /// joint velocities (5), `dt`: cycle period.
    pub fn torque(&mut self, model: &ArmModel, q: &[f64], qd: &[f64], dt: f64) -> Result<ImpOut> {
        let c = self.cfg.clone();
        if self.expires.is_some_and(|t| Instant::now() > t) && self.mode != Mode::Off {
            self.mode = Mode::Off;
            self.expires = None;
        }
        let (x, jb) = self.tool(model, q)?;
        let p = x.translation.vector;
        if self.pending_hold {
            self.pending_hold = false;
            self.p_target = p;
            self.r_target = Some(x.rotation);
            self.mode = Mode::Pose;
            self.fresh = true;
        }
        if self.fresh {
            self.q_cmd = q[..5].to_vec();
            self.p_cmd = p;
            self.r_cmd = x.rotation;
            self.fresh = false;
        }
        let k = self.stiffness;
        let qdv = DVector::from_column_slice(&qd[..5]);
        let mut tau = DVector::from_element(5, 0.0) - qdv.clone() * c.dq;
        match self.mode {
            Mode::Off => self.last_err_m = 0.0,
            Mode::Joints => {
                let step = c.max_joint_goal_speed_dps.to_radians() * dt;
                for i in 0..5 {
                    self.q_cmd[i] += (self.q_target[i] - self.q_cmd[i]).clamp(-step, step);
                    tau[i] += k * c.kq * (self.q_cmd[i] - q[i]);
                }
                self.last_err_m = 0.0;
            }
            Mode::Pose => {
                let d = self.p_target - self.p_cmd;
                let step = c.max_goal_speed * dt;
                self.p_cmd += if d.norm() > step { d * (step / d.norm()) } else { d };
                if let Some(rt) = self.r_target {
                    let max_a = c.max_goal_rot_speed * dt;
                    let a = self.r_cmd.angle_to(&rt);
                    self.r_cmd = if a > max_a { self.r_cmd.slerp(&rt, max_a / a) } else { rt };
                }
                let twist = &jb * &qdv;
                let v = Vector3::new(twist[0], twist[1], twist[2]);
                let w = Vector3::new(twist[3], twist[4], twist[5]);
                let e = self.p_cmd - p;
                self.last_err_m = e.norm();
                let mut f = e * (k * c.kp_lin) - v * c.kd_lin;
                if f.norm() > c.max_force {
                    f *= c.max_force / f.norm();
                }
                let mut mo = -w * c.kd_rot;
                if self.r_target.is_some() {
                    mo += (self.r_cmd * x.rotation.inverse()).scaled_axis() * (k * c.kp_rot);
                }
                if mo.norm() > c.max_moment {
                    mo *= c.max_moment / mo.norm();
                }
                let wrench = DVector::from_column_slice(&[f[0], f[1], f[2], mo[0], mo[1], mo[2]]);
                tau += jb.transpose() * wrench;
            }
        }
        let tau: Vec<f64> = tau.iter().map(|t| t.clamp(-c.max_joint_tau, c.max_joint_tau)).collect();
        Ok(ImpOut {
            tau,
            ee: p,
            mode_name: match self.mode {
                Mode::Off => "off",
                Mode::Joints => "joints",
                Mode::Pose => "pose",
            },
        })
    }

    /// Stream state to the last sender (for a policy's observations), at state_hz, for 2 s
    /// after its last message.
    pub fn send_state(&mut self, arm: &Arm, deg: &[f64], out: &ImpOut, x_quat: Option<UnitQuaternion<f64>>) {
        let (Some(sock), Some(peer)) = (&self.sock, self.peer) else { return };
        if self.last_rx.elapsed() > Duration::from_secs(2)
            || self.last_state_tx.elapsed().as_secs_f64() < 1.0 / self.cfg.state_hz
        {
            return;
        }
        self.last_state_tx = Instant::now();
        let quat = x_quat.map(|q| [q.w, q.i, q.j, q.k]);
        let msg = serde_json::json!({
            "arm": arm.name, "t": crate::control::unix_s(), "mode": out.mode_name,
            "q_deg": deg, "pos": [out.ee[0], out.ee[1], out.ee[2]], "quat": quat,
            "goal_pos": [self.p_cmd[0], self.p_cmd[1], self.p_cmd[2]], "err_m": self.last_err_m,
        });
        let _ = sock.send_to(msg.to_string().as_bytes(), peer);
    }

    pub fn tool_quat(&self, model: &ArmModel, q: &[f64]) -> Option<UnitQuaternion<f64>> {
        self.tool(model, q).ok().map(|(x, _)| x.rotation)
    }
}
