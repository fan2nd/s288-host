use crate::protocol::{build_control_packet, take_feedback, Feedback};
use serialport::SerialPort;
use std::collections::VecDeque;
use std::io::{Read, Write};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

pub const SERIAL_BAUD: u32 = 6_000_000;

#[derive(Clone, Copy, PartialEq)]
pub enum ControlMode {
    Position,
    Velocity,
    Torque,
}

#[derive(Clone, Copy)]
struct ActiveCommand {
    motor_id: u8,
    mode: ControlMode,
    target: f64,
    kp: f64,
    kd: f64,
}

#[derive(Clone)]
pub struct SerialManager {
    port: Arc<Mutex<Option<Box<dyn SerialPort>>>>,
    generation: Arc<AtomicU64>,
    pub connected: Arc<AtomicBool>,
    rx_log: Arc<Mutex<VecDeque<String>>>,
    latest: Arc<Mutex<Option<Feedback>>>,
    discovered: Arc<Mutex<Vec<u8>>>,
    active: Arc<Mutex<Option<ActiveCommand>>>,
}

impl SerialManager {
    pub fn new() -> Self {
        Self {
            port: Arc::new(Mutex::new(None)),
            generation: Arc::new(AtomicU64::new(0)),
            connected: Arc::new(AtomicBool::new(false)),
            rx_log: Arc::new(Mutex::new(VecDeque::with_capacity(64))),
            latest: Arc::new(Mutex::new(None)),
            discovered: Arc::new(Mutex::new(Vec::new())),
            active: Arc::new(Mutex::new(None)),
        }
    }

    pub fn connect(&self, path: &str) -> Result<(), String> {
        self.disconnect();
        let device = serialport::new(path, SERIAL_BAUD)
            .data_bits(serialport::DataBits::Eight)
            .flow_control(serialport::FlowControl::None)
            .parity(serialport::Parity::None)
            .stop_bits(serialport::StopBits::One)
            .timeout(Duration::from_millis(8))
            .open()
            .map_err(|err| format!("{} @ {}: {}", path, SERIAL_BAUD, err))?;

        if let Ok(mut port) = self.port.lock() {
            *port = Some(device);
        }
        if let Ok(mut ids) = self.discovered.lock() {
            ids.clear();
        }
        let generation = self.generation.fetch_add(1, Ordering::SeqCst) + 1;
        self.connected.store(true, Ordering::SeqCst);
        self.push_log(format!("LINK  {path}  /  {SERIAL_BAUD}  8N1"));

        let worker = self.clone();
        thread::Builder::new()
            .name("s288-serial-poll".to_string())
            .spawn(move || {
                // S288 的探测帧 Kp=Kd=0，不输出力矩；实际 ID 由总线反馈决定。
                while worker.connected.load(Ordering::SeqCst)
                    && worker.generation.load(Ordering::SeqCst) == generation
                {
                    if let Some(command) = worker.active_command() {
                        let _ = worker.transact_mode(
                            command.motor_id,
                            command.mode,
                            command.target,
                            command.kp,
                            command.kd,
                            1,
                        );
                        thread::sleep(Duration::from_millis(20));
                        continue;
                    }
                    for id in 0_u8..=15_u8 {
                        if worker.generation.load(Ordering::SeqCst) != generation {
                            return;
                        }
                        let _ = worker.transact(id, 0.0, 0.0, 0.0, 0.0, 0.0);
                    }
                    thread::sleep(Duration::from_millis(18));
                }
            })
            .map_err(|err| err.to_string())?;
        Ok(())
    }

    pub fn disconnect(&self) {
        self.connected.store(false, Ordering::SeqCst);
        self.generation.fetch_add(1, Ordering::SeqCst);
        if let Ok(mut port) = self.port.lock() {
            *port = None;
        }
        if let Ok(mut ids) = self.discovered.lock() {
            ids.clear();
        }
        self.clear_active();
    }

    pub fn send_control(
        &self,
        label: &str,
        motor_id: u8,
        mode: ControlMode,
        target: f64,
        kp: f64,
        kd: f64,
    ) -> Result<(), String> {
        let feedback = self.transact_mode(motor_id, mode, target, kp, kd, 1);
        if feedback.is_some() {
            if let Ok(mut active) = self.active.lock() {
                *active = Some(ActiveCommand {
                    motor_id,
                    mode,
                    target,
                    kp,
                    kd,
                });
            }
            self.push_log(format!("TX  {label}  /  ID{motor_id}  /  FE EE + CRC32"));
            Ok(())
        } else {
            Err("未收到有效 FC EE 反馈帧".to_string())
        }
    }

    fn transact(
        &self,
        motor_id: u8,
        torque: f64,
        speed: f64,
        position: f64,
        kp: f64,
        kd: f64,
    ) -> Option<Feedback> {
        self.transact_with_timeout(motor_id, torque, speed, position, kp, kd, 0)
    }

    fn transact_with_timeout(
        &self,
        motor_id: u8,
        torque: f64,
        speed: f64,
        position: f64,
        kp: f64,
        kd: f64,
        timeout_bit: u8,
    ) -> Option<Feedback> {
        let packet =
            build_control_packet(motor_id, 1, timeout_bit, torque, speed, position, kp, kd);
        let mut guard = self.port.lock().ok()?;
        let port = guard.as_mut()?;
        port.write_all(&packet).ok()?;
        port.flush().ok()?;

        let deadline = Instant::now() + Duration::from_millis(45);
        let mut stream = Vec::with_capacity(64);
        let mut chunk = [0_u8; 64];
        while Instant::now() < deadline {
            match port.read(&mut chunk) {
                Ok(size) if size > 0 => {
                    stream.extend_from_slice(&chunk[..size]);
                    if let Some(feedback) = take_feedback(&mut stream) {
                        if let Ok(mut latest) = self.latest.lock() {
                            *latest = Some(feedback.clone());
                        }
                        if let Ok(mut ids) = self.discovered.lock() {
                            if !ids.contains(&feedback.motor_id) {
                                ids.push(feedback.motor_id);
                                ids.sort_unstable();
                            }
                        }
                        self.push_log(format!(
                            "RX ID{} T{}°C {:.1}V P{:+.2}rad E{} W{} CRC OK",
                            feedback.motor_id,
                            feedback.temp,
                            feedback.voltage,
                            feedback.position,
                            feedback.error,
                            feedback.warning
                        ));
                        return Some(feedback);
                    }
                }
                Ok(_) => {}
                Err(err) if err.kind() == std::io::ErrorKind::TimedOut => {}
                Err(_) => break,
            }
        }
        None
    }

    pub fn take_logs(&self) -> Vec<String> {
        self.rx_log
            .lock()
            .map(|mut logs| logs.drain(..).collect())
            .unwrap_or_default()
    }

    pub fn latest(&self) -> Option<Feedback> {
        self.latest.lock().ok().and_then(|latest| latest.clone())
    }

    pub fn discovered_ids(&self) -> Vec<u8> {
        self.discovered
            .lock()
            .map(|ids| ids.clone())
            .unwrap_or_default()
    }

    fn active_command(&self) -> Option<ActiveCommand> {
        self.active.lock().ok().and_then(|command| *command)
    }

    fn clear_active(&self) {
        if let Ok(mut active) = self.active.lock() {
            *active = None;
        }
    }

    fn transact_mode(
        &self,
        motor_id: u8,
        mode: ControlMode,
        target: f64,
        kp: f64,
        kd: f64,
        timeout_bit: u8,
    ) -> Option<Feedback> {
        let (torque, speed, position, kp, kd) = match mode {
            ControlMode::Position => (0.0, 0.0, target.to_radians(), kp, kd),
            ControlMode::Velocity => (0.0, target.to_radians(), 0.0, 0.0, kd),
            ControlMode::Torque => (target, 0.0, 0.0, 0.0, 0.0),
        };
        self.transact_with_timeout(motor_id, torque, speed, position, kp, kd, timeout_bit)
    }

    fn push_log(&self, message: String) {
        if let Ok(mut logs) = self.rx_log.lock() {
            if logs.len() >= 64 {
                logs.pop_front();
            }
            logs.push_back(message);
        }
    }
}

pub fn scan_serial_ports() -> Vec<String> {
    let mut ports = serialport::available_ports()
        .map(|ports| {
            ports
                .into_iter()
                .map(|port| port.port_name)
                .collect::<Vec<_>>()
        })
        .unwrap_or_default();
    ports.sort();
    ports
}
