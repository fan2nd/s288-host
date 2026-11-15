use crate::serial::{scan_serial_ports, ControlMode, SerialManager};
use dioxus::prelude::*;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::time::Duration;

const STYLE: &str = include_str!("../assets/style.css");

fn mode_label(mode: ControlMode) -> &'static str {
    match mode {
        ControlMode::Position => "POSITION",
        ControlMode::Velocity => "VELOCITY",
        ControlMode::Torque => "TORQUE",
    }
}

fn mode_unit(mode: ControlMode) -> &'static str {
    match mode {
        ControlMode::Position => "°",
        ControlMode::Velocity => "°/s",
        ControlMode::Torque => "N·m",
    }
}

fn mode_min(mode: ControlMode) -> f64 {
    match mode {
        ControlMode::Position => -180.0,
        ControlMode::Velocity => -60.0,
        ControlMode::Torque => -3.0,
    }
}

fn mode_max(mode: ControlMode) -> f64 {
    match mode {
        ControlMode::Position => 180.0,
        ControlMode::Velocity => 60.0,
        ControlMode::Torque => 3.0,
    }
}

fn mode_step(mode: ControlMode) -> f64 {
    match mode {
        ControlMode::Position | ControlMode::Velocity => 0.1,
        ControlMode::Torque => 0.01,
    }
}

fn position_degrees(radians: f32) -> f32 {
    radians.to_degrees()
}

fn speed_degrees(radians_per_second: f32) -> f32 {
    radians_per_second.to_degrees()
}

fn live_control_handler(
    serial: Arc<SerialManager>,
    mut last_command: Signal<String>,
    motor_id: Signal<u8>,
    mode: Signal<ControlMode>,
    mut target: Signal<f64>,
    kp: Signal<f64>,
    kd: Signal<f64>,
) -> impl FnMut(Event<FormData>) {
    move |event| {
        let current_mode = mode();
        let value = event
            .value()
            .parse::<f64>()
            .unwrap_or(0.0)
            .clamp(mode_min(current_mode), mode_max(current_mode));
        target.set(value);
        if serial.connected.load(Ordering::SeqCst) {
            match serial.send_control("LIVE", motor_id(), current_mode, value as f64, kp(), kd()) {
                Ok(()) => last_command.set(format!(
                    "LIVE {} ID{} / FC EE OK",
                    mode_label(current_mode),
                    motor_id()
                )),
                Err(error) => last_command.set(format!("SERIAL ERROR / {error}")),
            }
        }
    }
}

fn live_gain_handler(
    serial: Arc<SerialManager>,
    mut last_command: Signal<String>,
    motor_id: Signal<u8>,
    mode: Signal<ControlMode>,
    target: Signal<f64>,
    mut kp: Signal<f64>,
    mut kd: Signal<f64>,
    is_kp: bool,
) -> impl FnMut(Event<FormData>) {
    move |event| {
        let value = event
            .value()
            .parse::<f64>()
            .unwrap_or(if is_kp { 10.0 } else { 0.5 });
        if is_kp {
            kp.set(value);
        } else {
            kd.set(value);
        }
        if serial.connected.load(Ordering::SeqCst) {
            match serial.send_control("LIVE GAIN", motor_id(), mode(), target() as f64, kp(), kd())
            {
                Ok(()) => last_command.set(format!("LIVE GAIN ID{} / FC EE OK", motor_id())),
                Err(error) => last_command.set(format!("SERIAL ERROR / {error}")),
            }
        }
    }
}

fn mode_handler(
    serial: Arc<SerialManager>,
    mut last_command: Signal<String>,
    motor_id: Signal<u8>,
    mut mode: Signal<ControlMode>,
    mut target: Signal<f64>,
    kp: Signal<f64>,
    kd: Signal<f64>,
    next_mode: ControlMode,
) -> impl FnMut(Event<MouseData>) {
    move |_| {
        mode.set(next_mode);
        target.set(0.0);
        if serial.connected.load(Ordering::SeqCst) {
            match serial.send_control("MODE", motor_id(), next_mode, 0.0, kp(), kd()) {
                Ok(()) => last_command.set(format!(
                    "MODE {} ID{} / FC EE OK",
                    mode_label(next_mode),
                    motor_id()
                )),
                Err(error) => last_command.set(format!("SERIAL ERROR / {error}")),
            }
        }
    }
}

pub fn app() -> Element {
    let serial = Arc::new(use_hook(SerialManager::new));
    let mut connected = use_signal(|| false);
    let mut ports = use_signal(scan_serial_ports);
    let mut selected_port = use_signal(|| scan_serial_ports().first().cloned().unwrap_or_default());
    let mut motor_id = use_signal(|| 0_u8);
    let mut known_ids = use_signal(Vec::<u8>::new);
    let control_mode = use_signal(|| ControlMode::Position);
    let target_position = use_signal(|| 0.0_f64);
    let kp = use_signal(|| 10.0_f64);
    let kd = use_signal(|| 0.5_f64);
    let mut last_command = use_signal(|| "READY".to_string());
    let mut rx_count = use_signal(|| 0_u32);
    let mut feedback_id = use_signal(|| 0_u8);
    let mut position = use_signal(|| 0.0_f32);
    let mut speed = use_signal(|| 0.0_f32);
    let mut torque = use_signal(|| 0.0_f32);
    let mut temperature = use_signal(|| 0_i8);
    let mut voltage = use_signal(|| 0.0_f32);
    let mut error = use_signal(|| 0_u32);
    let mut warning = use_signal(|| 0_u8);

    {
        let serial = serial.clone();
        use_future(move || {
            let serial = serial.clone();
            async move {
                loop {
                    connected.set(serial.connected.load(Ordering::SeqCst));
                    known_ids.set(serial.discovered_ids());
                    for message in serial.take_logs() {
                        last_command.set(message);
                        rx_count += 1;
                    }
                    if let Some(feedback) = serial.latest() {
                        feedback_id.set(feedback.motor_id);
                        position.set(feedback.position);
                        speed.set(feedback.speed);
                        torque.set(feedback.torque);
                        temperature.set(feedback.temp);
                        voltage.set(feedback.voltage);
                        error.set(feedback.error);
                        warning.set(feedback.warning);
                    }
                    tokio::time::sleep(Duration::from_millis(70)).await;
                }
            }
        });
    }

    let serial_for_connection = serial.clone();

    rsx! {
        style { "{STYLE}" }
        div { class: "app",
            aside { class: "sidebar",
                div { class: "brand",
                    div { class: "brand-mark" }
                    div { class: "brand-copy",
                        div { class: "brand-name", "S288" }
                        div { class: "brand-sub", "SERVO HOST" }
                    }
                }
                div { class: "eyebrow side-heading", "MENU" }
                nav { class: "nav",
                    button { class: "nav-item active", "◈", span { "Servo console" } }
                }
                div { class: "side-bottom",
                    div { class: "eyebrow side-heading", "BUS" }
                    div { class: "side-card",
                        div { class: "side-card-row", span { class: "side-label", "Baudrate" }, span { class: "side-value", "6 Mbps" } }
                        div { class: "side-card-row", span { class: "side-label", "Format" }, span { class: "side-value", "8N1" } }
                        div { class: "side-card-row", span { class: "side-label", "TX / RX" }, span { class: "side-value", "20 / 26 B" } }
                    }
                }
            }
            main { class: "main",
                header { class: "topbar",
                    div { div { class: "eyebrow", "UNITREE S288" }, h1 { class: "page-title", "Servo console" } }
                    div { class: "top-actions",
                        select { class: "port-select", value: "{selected_port()}", onchange: move |event| selected_port.set(event.value()),
                            if ports().is_empty() { option { value: "", "NO SERIAL PORT" } }
                            for port in ports() { option { value: "{port}", "{port}" } }
                        }
                        div { class: "connection",
                            span { class: "status-dot", style: if connected() { "" } else { "background: var(--red); box-shadow: none;" } }
                            span { if connected() { "6 MBPS ONLINE" } else { "SERIAL OFFLINE" } }
                        }
                        button { class: "connect-btn", onclick: move |_| {
                            if serial_for_connection.connected.load(Ordering::SeqCst) {
                                serial_for_connection.disconnect();
                                connected.set(false);
                                last_command.set("SERIAL CLOSED".to_string());
                            } else if selected_port().is_empty() {
                                last_command.set("NO SERIAL PORT FOUND".to_string());
                            } else {
                                match serial_for_connection.connect(&selected_port()) {
                                    Ok(()) => last_command.set("SERIAL OPEN".to_string()),
                                    Err(error) => last_command.set(format!("OPEN FAILED / {error}")),
                                }
                            }
                        }, if connected() { "DISCONNECT" } else { "CONNECT" } }
                        button { class: "round-btn", title: "Refresh ports", onclick: move |_| {
                            let found = scan_serial_ports();
                            if selected_port().is_empty() {
                                if let Some(port) = found.first() { selected_port.set(port.clone()); }
                            }
                            ports.set(found);
                        }, "↻" }
                    }
                }
                div { class: "grid",
                    section { class: "panel servo-panel",
                        div { class: "panel-head",
                            div { class: "panel-title", "SETTINGS · ID {motor_id()}" }
                            div { class: "eyebrow", if feedback_id() == motor_id() { "ONLINE" } else { "WAITING" } }
                        }
                        div { class: "servo-row",
                            div { class: "servo-readout", div { class: "eyebrow", "TARGET" }, strong { "{target_position():+.1}" }, span { "{mode_unit(control_mode())}" } }
                            div { class: "state-grid",
                                div { span { "SERVO ID" }, strong { "{motor_id()}" } }
                                div { span { "MODE" }, strong { "{mode_label(control_mode())}" } }
                                div { span { "KP / KD" }, strong { "{kp():.2} / {kd():.2}" } }
                                div { span { "REALTIME" }, strong { "ON" } }
                            }
                        }
                        div { class: "servo-footer",
                            div { class: "id-switch",
                                if known_ids().is_empty() {
                                    span { class: "id-empty", "NO DISCOVERED ID" }
                                }
                                for id in known_ids() {
                                    button { class: if motor_id() == id { "id-btn active" } else { "id-btn" }, onclick: move |_| motor_id.set(id), "ID {id}" }
                                }
                            }
                            div { class: "packet-count", "VALID RX  ", strong { "{rx_count()}" } }
                        }
                    }
                    section { class: "panel telemetry",
                        div { class: "panel-head", div { class: "panel-title", "FEEDBACK · ID {feedback_id()}" }, div { class: "eyebrow", if feedback_id() == motor_id() { "OK" } else { "--" } } }
                        div { class: "telemetry-stack",
                            div { class: "telemetry-row", div { div { class: "telemetry-name", "Output position" }, div { class: "telemetry-value", "{position_degrees(position()):+.1}", span { class: "telemetry-unit", "°" } } } }
                            div { class: "telemetry-row", div { div { class: "telemetry-name", "Output velocity" }, div { class: "telemetry-value", "{speed_degrees(speed()):+.1}", span { class: "telemetry-unit", "°/s" } } } }
                            div { class: "telemetry-row", div { div { class: "telemetry-name", "Output torque" }, div { class: "telemetry-value", "{torque():+.2}", span { class: "telemetry-unit", "N·m" } } } }
                            div { class: "telemetry-row compact", div { span { class: "telemetry-name", "Housing" }, strong { "{temperature()}°C" } }, div { span { class: "telemetry-name", "Bus" }, strong { "{voltage():.1} V" } } }
                            div { class: "telemetry-row compact", div { span { class: "telemetry-name", "Error" }, strong { if error() == 0 { "NONE" } else { "{error()}" } } }, div { span { class: "telemetry-name", "Warning" }, strong { if warning() == 0 { "NONE" } else { "{warning()}" } } } }
                        }
                    }
                    section { class: "panel control-panel",
                        div { class: "panel-head", div { class: "panel-title", "CONTROL · {mode_label(control_mode())}" }, div { class: "eyebrow", "LIVE" } }
                        div { class: "control-layout",
                            div { class: "control-group",
                            div { class: "control-label", "SERVO ID" }
                                input { class: "id-input", r#type: "number", min: "0", max: "15", value: "{motor_id()}", oninput: move |event| motor_id.set(event.value().parse().unwrap_or(0).clamp(0, 15)) }
                                div { class: "mode-switch",
                                    for mode in [ControlMode::Position, ControlMode::Velocity, ControlMode::Torque] {
                                        button { class: if control_mode() == mode { "mode-btn active" } else { "mode-btn" }, onclick: mode_handler(serial.clone(), last_command, motor_id, control_mode, target_position, kp, kd, mode), "{mode_label(mode)}" }
                                    }
                                }
                                div { class: "control-label", "TARGET  ·  {target_position():.3} {mode_unit(control_mode())}" }
                                input { class: "range position-range", r#type: "range", min: "{mode_min(control_mode())}", max: "{mode_max(control_mode())}", step: "{mode_step(control_mode())}", value: "{target_position():.3}", oninput: live_control_handler(serial.clone(), last_command, motor_id, control_mode, target_position, kp, kd) }
                                div { class: "range-ends", span { "{mode_min(control_mode()):.3} {mode_unit(control_mode())}" }, span { "0" }, span { "+{mode_max(control_mode()):.3} {mode_unit(control_mode())}" } }
                            }
                            div { class: "control-group gain-group",
                                div { class: "control-label", "KP  ·  {kp():.1}" }
                                input { class: "range", r#type: "range", min: "0", max: "10", step: "0.01", value: "{kp()}", oninput: live_gain_handler(serial.clone(), last_command, motor_id, control_mode, target_position, kp, kd, true) }
                                div { class: "range-ends", span { "0" }, span { "10" } }
                                div { class: "control-label", "KD  ·  {kd():.3}" }
                                input { class: "range", r#type: "range", min: "0", max: "10", step: "0.01", value: "{kd()}", oninput: live_gain_handler(serial.clone(), last_command, motor_id, control_mode, target_position, kp, kd, false) }
                                div { class: "range-ends", span { "0" }, span { "10" } }
                            }
                            div { class: "realtime-note", span { class: "status-dot" }, "LIVE" }
                        }
                        div { class: "command-log", span { "LAST EVENT" }, strong { "{last_command()}" } }
                    }
                }
            }
        }
    }
}
