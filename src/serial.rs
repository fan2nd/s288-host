use crate::protocol::{build_control_packet, take_feedback, Feedback};
use serialport::SerialPort;
use std::io::{Read, Write};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

pub const SERIAL_BAUD: u32 = 6_000_000;

#[derive(Clone, Copy, Default, PartialEq, Debug)]
pub struct Setpoint {
    pub position_deg: f64,
    pub velocity_deg_s: f64,
    pub torque_nm: f64,
    pub kp: f64,
    pub kd: f64,
}

#[derive(Clone, Copy)]
struct ActiveCommand {
    motor_id: u8,
    setpoint: Setpoint,
}

#[derive(Clone)]
pub struct SerialManager {
    port: Arc<Mutex<Option<Box<dyn SerialPort>>>>,
    generation: Arc<AtomicU64>,
    pub connected: Arc<AtomicBool>,
    rx_count: Arc<AtomicU64>,
    latest: Arc<Mutex<Option<(Feedback, Instant)>>>,
    discovered: Arc<Mutex<Vec<u8>>>,
    active: Arc<Mutex<Option<ActiveCommand>>>,
}

impl SerialManager {
    pub fn new() -> Self {
        Self {
            port: Arc::new(Mutex::new(None)),
            generation: Arc::new(AtomicU64::new(0)),
            connected: Arc::new(AtomicBool::new(false)),
            rx_count: Arc::new(AtomicU64::new(0)),
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

        let worker = self.clone();
        thread::Builder::new()
            .name("s288-serial-poll".to_string())
            .spawn(move || {
                // S288 的探测帧 Kp=Kd=0，不输出力矩；实际 ID 由总线反馈决定。
                while worker.connected.load(Ordering::SeqCst)
                    && worker.generation.load(Ordering::SeqCst) == generation
                {
                    for id in 0..=15 {
                        let Ok(active) = worker.active.lock() else {
                            return;
                        };
                        if worker.generation.load(Ordering::SeqCst) != generation {
                            return;
                        }
                        if let Some(command) = *active {
                            worker.transact(command.motor_id, command.setpoint, 1);
                            break;
                        }
                        worker.transact(id, Setpoint::default(), 0);
                    }
                    thread::sleep(Duration::from_millis(18));
                }
            })
            .map_err(|err| {
                self.disconnect();
                err.to_string()
            })?;
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
        if let Ok(mut active) = self.active.lock() {
            *active = None;
        }
        if let Ok(mut latest) = self.latest.lock() {
            *latest = None;
        }
        self.rx_count.store(0, Ordering::Relaxed);
    }

    pub fn send_control(&self, motor_id: u8, setpoint: Setpoint) -> Result<(), ()> {
        let mut active = self.active.lock().map_err(|_| ())?;
        self.transact(motor_id, setpoint, 1).ok_or(())?;
        *active = Some(ActiveCommand { motor_id, setpoint });
        Ok(())
    }

    pub fn stop(&self, selected_id: u8) -> Result<u8, ()> {
        let mut active = self.active.lock().map_err(|_| ())?;
        let id = active.map_or(selected_id, |command| command.motor_id);
        let neutral = Setpoint::default();
        *active = Some(ActiveCommand {
            motor_id: id,
            setpoint: neutral,
        });
        self.transact(id, neutral, 1).map(|_| id).ok_or(())
    }

    fn transact(&self, motor_id: u8, setpoint: Setpoint, timeout: u8) -> Option<Feedback> {
        let packet = build_control_packet(
            motor_id,
            1,
            timeout,
            setpoint.torque_nm,
            setpoint.velocity_deg_s.to_radians(),
            setpoint.position_deg.to_radians(),
            setpoint.kp,
            setpoint.kd,
        );
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
                    while let Some(feedback) = take_feedback(&mut stream) {
                        if let Ok(mut latest) = self.latest.lock() {
                            *latest = Some((feedback.clone(), Instant::now()));
                        }
                        if let Ok(mut ids) = self.discovered.lock() {
                            if !ids.contains(&feedback.motor_id) {
                                ids.push(feedback.motor_id);
                                ids.sort_unstable();
                            }
                        }
                        self.rx_count.fetch_add(1, Ordering::Relaxed);
                        if feedback.motor_id != motor_id {
                            continue;
                        }
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

    pub fn rx_count(&self) -> u64 {
        self.rx_count.load(Ordering::Relaxed)
    }

    pub fn latest(&self) -> Option<Feedback> {
        let latest = self.latest.lock().ok()?;
        let (feedback, at) = latest.as_ref()?;
        (at.elapsed() < Duration::from_secs(2)).then(|| feedback.clone())
    }

    pub fn discovered_ids(&self) -> Vec<u8> {
        self.discovered
            .lock()
            .map(|ids| ids.clone())
            .unwrap_or_default()
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stop_replaces_active_command_even_without_feedback() {
        let manager = SerialManager::new();
        *manager.active.lock().unwrap() = Some(ActiveCommand {
            motor_id: 7,
            setpoint: Setpoint {
                torque_nm: 1.0,
                ..Default::default()
            },
        });
        assert!(manager.stop(2).is_err());
        let stopped = manager.active.lock().unwrap().unwrap();
        assert_eq!(stopped.motor_id, 7);
        assert_eq!(stopped.setpoint, Setpoint::default());
    }

    #[cfg(unix)]
    #[test]
    fn stop_sends_neutral_frame_to_active_motor() {
        use crate::protocol::{crc32_core, CMD_LEN, FB_LEN};
        let manager = SerialManager::new();
        let (mut device, mut host) = serialport::TTYPort::pair().unwrap();
        device.set_timeout(Duration::from_secs(1)).unwrap();
        host.set_timeout(Duration::from_millis(8)).unwrap();
        *manager.port.lock().unwrap() = Some(Box::new(host));
        let device_task = thread::spawn(move || {
            for torque in [1.0, 0.0] {
                let mut packet = [0; CMD_LEN];
                device.read_exact(&mut packet).unwrap();
                assert_eq!(
                    packet,
                    build_control_packet(7, 1, 1, torque, 0.0, 0.0, 0.0, 0.0)
                );
                let mut feedback = [0; FB_LEN];
                feedback[..3].copy_from_slice(&[0xfc, 0xee, 7]);
                let crc = crc32_core(&feedback[2..22]);
                feedback[22..].copy_from_slice(&crc.to_le_bytes());
                device.write_all(&feedback).unwrap();
                // PTY writes are unbuffered. Draining both ends can deadlock on macOS.
            }
            // Keep the peer open until the last response has been consumed.
            thread::sleep(Duration::from_millis(50));
        });
        manager
            .send_control(
                7,
                Setpoint {
                    torque_nm: 1.0,
                    ..Default::default()
                },
            )
            .unwrap();
        assert_eq!(manager.stop(2), Ok(7));
        device_task.join().unwrap();
    }
}
