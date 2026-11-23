//! Feetech STS3215 access for gravity compensation, on top of `rustypot`.
//!
//! The STS3215 has no current (torque) mode. It has an open-loop PWM mode
//! (`Operating_Mode` = 2): the duty cycle goes in `Goal_Time` (address 44) as
//! sign-magnitude, magnitude 0..1000 (= 0..100 %), direction in bit 10. At a fixed
//! supply voltage and low speed, motor torque is close to proportional to duty cycle,
//! so PWM is used as the torque command (see `control.rs` for the conversion).

use std::time::Duration;

use anyhow::{Context, Result, anyhow, bail};
use rustypot::servo::feetech::sts3215::Sts3215Controller;

// Register addresses (STS3215 control table, same as rustypot / LeRobot).
pub const ADDR_OPERATING_MODE: u8 = 33;
pub const ADDR_TORQUE_ENABLE: u8 = 40;
pub const ADDR_GOAL_POSITION: u8 = 42;
pub const ADDR_GOAL_TIME: u8 = 44; // PWM command in PWM mode
pub const ADDR_TORQUE_LIMIT: u8 = 48; // RAM, caps the output in every mode
pub const ADDR_PRESENT_POSITION: u8 = 56; // 56 pos(2) 58 speed(2) 60 load(2)
pub const ADDR_PRESENT_VOLTAGE: u8 = 62;
pub const ADDR_PRESENT_TEMPERATURE: u8 = 63;

pub const MODE_POSITION: u8 = 0;
pub const MODE_PWM: u8 = 2;
pub const PWM_MAX: i32 = 1000;
pub const STEPS_PER_REV: f64 = 4096.0;

/// Decode a sign-magnitude register value with the sign in `bit`.
pub fn sm_decode(raw: u16, bit: u8) -> i32 {
    let mag = (raw & ((1u16 << bit) - 1)) as i32;
    if raw & (1u16 << bit) != 0 { -mag } else { mag }
}

/// Encode a signed value as sign-magnitude with the sign in `bit`.
pub fn sm_encode(value: i32, bit: u8) -> u16 {
    let mag = value.unsigned_abs().min((1u32 << bit) - 1) as u16;
    if value < 0 { mag | (1u16 << bit) } else { mag }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct RawState {
    /// Present position, raw steps 0..4095 (homing offset already applied by the motor).
    pub pos: i32,
    /// Present speed, steps/s, signed.
    pub speed: i32,
    /// Present load, 0.1 % of max output, signed.
    pub load: i32,
}

impl RawState {
    pub fn speed_radps(&self) -> f64 {
        self.speed as f64 * 2.0 * std::f64::consts::PI / STEPS_PER_REV
    }
}

pub struct Bus {
    pub port: String,
    c: Sts3215Controller,
}

impl Bus {
    pub fn open(port: &str) -> Result<Self> {
        let sp = serialport::new(port, 1_000_000)
            .timeout(Duration::from_millis(20))
            .open()
            .with_context(|| format!("open {port} (is the XLeRobot host/bridge still running? is the user in 'dialout'?)"))?;
        let c = Sts3215Controller::new().with_protocol_v1().with_serial_port(sp);
        Ok(Self { port: port.to_string(), c })
    }

    pub fn ping(&mut self, id: u8) -> bool {
        self.c.ping(id).unwrap_or(false)
    }

    fn err(e: Box<dyn std::error::Error>) -> anyhow::Error {
        anyhow!("{e}")
    }

    pub fn read_u8(&mut self, id: u8, addr: u8) -> Result<u8> {
        let v = self.c.read_raw_data(id, addr, 1).map_err(Self::err)?;
        Ok(v[0])
    }

    pub fn read_u16(&mut self, id: u8, addr: u8) -> Result<u16> {
        let v = self.c.read_raw_data(id, addr, 2).map_err(Self::err)?;
        Ok(u16::from_le_bytes([v[0], v[1]]))
    }

    pub fn write_u8(&mut self, id: u8, addr: u8, v: u8) -> Result<()> {
        self.c.write_raw_data(id, addr, vec![v]).map_err(Self::err)
    }

    pub fn write_u16(&mut self, id: u8, addr: u8, v: u16) -> Result<()> {
        self.c.write_raw_data(id, addr, v.to_le_bytes().to_vec()).map_err(Self::err)
    }

    /// One sync-read transaction: position, speed and load of every id.
    pub fn read_state(&mut self, ids: &[u8]) -> Result<Vec<RawState>> {
        let rows = self
            .c
            .sync_read_raw_data(ids, ADDR_PRESENT_POSITION, 6)
            .map_err(Self::err)?;
        if rows.len() != ids.len() {
            bail!("sync read returned {} of {} motors", rows.len(), ids.len());
        }
        Ok(rows
            .iter()
            .map(|b| RawState {
                pos: sm_decode(u16::from_le_bytes([b[0], b[1]]), 15),
                speed: sm_decode(u16::from_le_bytes([b[2], b[3]]), 15),
                load: sm_decode(u16::from_le_bytes([b[4], b[5]]), 10),
            })
            .collect())
    }

    /// One sync-write transaction (no reply): PWM duty, -1000..1000, per id.
    pub fn write_pwm(&mut self, ids: &[u8], pwm: &[i32]) -> Result<()> {
        let data: Vec<Vec<u8>> = pwm
            .iter()
            .map(|&p| sm_encode(p.clamp(-PWM_MAX, PWM_MAX), 10).to_le_bytes().to_vec())
            .collect();
        self.c
            .sync_write_raw_data(ids, ADDR_GOAL_TIME, &data)
            .map_err(Self::err)
    }

    /// Torque on closes the EEPROM lock, torque off opens it (LeRobot/rustypot convention).
    /// Retry a bus operation a few times; a motor can miss one packet.
    fn retry<T>(&mut self, what: &str, id: u8, mut f: impl FnMut(&mut Self) -> Result<T>) -> Result<T> {
        let mut last = None;
        for _ in 0..4 {
            match f(self) {
                Ok(v) => return Ok(v),
                Err(e) => {
                    last = Some(e);
                    std::thread::sleep(Duration::from_millis(5));
                }
            }
        }
        Err(last.unwrap()).with_context(|| format!("motor {id}: {what}"))
    }

    /// Torque on/off. The EEPROM lock is never touched, so mode changes stay in RAM and
    /// a power cycle always brings the motor back in its saved (position) mode.
    pub fn set_torque(&mut self, id: u8, on: bool) -> Result<()> {
        self.retry(if on { "torque on" } else { "torque off" }, id, |b| b.write_u8(id, ADDR_TORQUE_ENABLE, on as u8))
    }

    /// Change operating mode WITHOUT turning torque off (a loaded joint would fall), then
    /// read it back.
    pub fn set_mode(&mut self, id: u8, mode: u8) -> Result<()> {
        self.retry("write mode", id, |b| b.write_u8(id, ADDR_OPERATING_MODE, mode))?;
        let back = self.retry("read mode", id, |b| b.read_u8(id, ADDR_OPERATING_MODE))?;
        if back != mode {
            bail!("motor {id}: wrote operating mode {mode}, reads back {back} (mode not writable with the EEPROM locked?)");
        }
        Ok(())
    }

    /// PWM mode with zero output and a PWM cap; torque on. Order: zero PWM and cap first,
    /// so the motor never sees a stale Goal_Time when the mode flips.
    pub fn enter_pwm(&mut self, ids: &[u8], torque_limit: &[u16]) -> Result<()> {
        for (&id, &lim) in ids.iter().zip(torque_limit) {
            self.retry("zero pwm", id, |b| b.write_u16(id, ADDR_GOAL_TIME, 0))?;
            self.retry("torque limit", id, |b| b.write_u16(id, ADDR_TORQUE_LIMIT, lim))?;
            self.set_mode(id, MODE_PWM)?;
            self.set_torque(id, true)?;
        }
        Ok(())
    }

    /// Safe exit: zero PWM, goal = present position, position mode, torque on. The goal is
    /// set before the mode flips, so the joint holds where it is with no free fall.
    /// Tries every motor even if one fails.
    pub fn hold_here(&mut self, ids: &[u8], torque_limit: u16) -> Result<()> {
        let _ = self.write_pwm(ids, &vec![0; ids.len()]);
        let mut first_err = None;
        for &id in ids {
            let r = (|| -> Result<()> {
                let pos = self.retry("read position", id, |b| b.read_u16(id, ADDR_PRESENT_POSITION))?;
                self.retry("goal position", id, |b| b.write_u16(id, ADDR_GOAL_POSITION, pos))?;
                self.retry("torque limit", id, |b| b.write_u16(id, ADDR_TORQUE_LIMIT, torque_limit))?;
                self.set_mode(id, MODE_POSITION)?;
                self.set_torque(id, true)
            })();
            if let Err(e) = r {
                eprintln!("[{}] hold failed: {e:#}", self.port);
                first_err.get_or_insert(e);
            }
        }
        first_err.map_or(Ok(()), Err)
    }

    /// Emergency: zero PWM, position mode, torque off. The arm WILL fall.
    pub fn release(&mut self, ids: &[u8]) -> Result<()> {
        let _ = self.write_pwm(ids, &vec![0; ids.len()]);
        for &id in ids {
            let _ = self.write_u16(id, ADDR_GOAL_TIME, 0);
            self.set_torque(id, false)?;
            self.set_mode(id, MODE_POSITION)?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sign_magnitude_roundtrip() {
        for v in [-1000, -1, 0, 1, 512, 1000] {
            assert_eq!(sm_decode(sm_encode(v, 10), 10), v);
        }
        assert_eq!(sm_encode(-300, 10), 300 | 1024);
        assert_eq!(sm_decode(0x8000 | 25, 15), -25);
        // Saturates instead of spilling into the sign bit.
        assert_eq!(sm_encode(5000, 10), 1023);
    }
}
