use crate::protocol::Feedback;
use crate::serial::{scan_serial_ports, SerialManager, Setpoint};
use dioxus::prelude::*;
use std::time::Duration;

const STYLE: &str = include_str!("../assets/style.css");

#[derive(Clone, Copy, PartialEq, Debug)]
enum Mode {
    Position,
    Velocity,
    Torque,
    Mit,
}

impl Mode {
    fn label(self) -> &'static str {
        match self {
            Self::Position => "位置",
            Self::Velocity => "速度",
            Self::Torque => "力矩",
            Self::Mit => "MIT",
        }
    }
    fn fields(self) -> &'static [usize] {
        match self {
            Self::Position => &[0, 3, 4],
            Self::Velocity => &[1, 4],
            Self::Torque => &[2],
            Self::Mit => &[0, 1, 2, 3, 4],
        }
    }
}

struct Field(&'static str, &'static str, f64, f64, f64);
const FIELDS: [Field; 5] = [
    Field("目标位置", "°", -180., 180., 0.1),
    Field("目标速度", "°/s", -60., 60., 0.1),
    Field("前馈力矩", "N·m", -3., 3., 0.01),
    Field("位置增益 Kp", "N·m/rad", 0., 10., 0.01),
    Field("阻尼增益 Kd", "(N·m)/(rad/s)", 0., 10., 0.01),
];
fn defaults() -> [String; 5] {
    ["0", "0", "0", "10", "0.5"].map(str::to_owned)
}

fn setpoint(mode: Mode, draft: &[String; 5]) -> Result<Setpoint, Vec<usize>> {
    let mut values = [0.; 5];
    let mut invalid = Vec::new();
    for &i in mode.fields() {
        match draft[i].parse::<f64>() {
            Ok(v) if (FIELDS[i].2..=FIELDS[i].3).contains(&v) => values[i] = v,
            _ => invalid.push(i),
        }
    }
    if !invalid.is_empty() {
        return Err(invalid);
    }
    Ok(Setpoint {
        position_deg: values[0],
        velocity_deg_s: values[1],
        torque_nm: values[2],
        kp: values[3],
        kd: values[4],
    })
}

fn show_error(mut message: Signal<String>, text: impl Into<String>, focus: &str) {
    message.set(text.into());
    document::eval(&format!("const d=document.querySelector('dialog'); d.onclose=()=>document.getElementById('{focus}')?.focus(); if(!d.open)d.showModal();"));
}

pub fn app() -> Element {
    let serial = use_hook(SerialManager::new);
    let mut connected = use_signal(|| false);
    let mut ports = use_signal(scan_serial_ports);
    let mut port = use_signal(|| ports.read().first().cloned().unwrap_or_default());
    let mut id = use_signal(|| 0u8);
    let mut ids = use_signal(Vec::<u8>::new);
    let mut mode = use_signal(|| Mode::Position);
    let mut basic = use_signal(|| Mode::Position);
    let standard_values = use_signal(defaults);
    let mit_values = use_signal(defaults);
    let mut draft = if mode() == Mode::Mit {
        mit_values
    } else {
        standard_values
    };
    let mut feedback = use_signal(|| None::<Feedback>);
    let mut received = use_signal(|| 0u64);
    let mut status = use_signal(|| "未应用控制".to_owned());
    let message = use_signal(String::new);

    let reader = serial.clone();
    use_future(move || {
        let reader = reader.clone();
        async move {
            loop {
                feedback.set(reader.latest());
                received.set(reader.rx_count());
                ids.set(reader.discovered_ids());
                tokio::time::sleep(Duration::from_millis(150)).await;
            }
        }
    });

    use_effect(|| {
        document::eval(
            r#"
            if (!window.s288StopKey) {
                window.s288StopKey = e => {
                    if(e.code !== 'Space' || document.getElementById('stop')?.disabled) return;
                    e.preventDefault();
                    e.stopImmediatePropagation();
                    if(e.type === 'keydown' && !e.repeat) document.getElementById('stop').click();
                };
                window.addEventListener('keydown',window.s288StopKey,true);
                window.addEventListener('keyup',window.s288StopKey,true);
            }
        "#,
        );
    });

    let link = serial.clone();
    let stopper = serial.clone();
    let stop_output = use_callback(move |()| match stopper.stop(id()) {
        Ok(id) => status.set(format!("ID {id} · 已切换至零力矩")),
        Err(()) => {
            status.set("已取消旧指令 · 停止未确认".into());
            show_error(
                message,
                "未收到停止反馈。正在重复发送零力矩指令，请检查设备状态。",
                "stop",
            );
        }
    });
    let switch_mode = use_callback(move |next: Mode| {
        if next == mode() {
            return;
        }
        if connected() {
            stop_output.call(());
        }
        if next != Mode::Mit {
            basic.set(next);
        }
        mode.set(next);
    });
    let fb = if connected() { feedback() } else { None };
    let feedback_state = fb
        .as_ref()
        .map(|f| format!("ID {}", f.motor_id))
        .unwrap_or_else(|| {
            if connected() {
                "等待反馈".into()
            } else {
                "未连接".into()
            }
        });
    let measurements = [
        (
            "输出位置",
            "°",
            fb.as_ref()
                .map(|f| format!("{:+.1}", f.position.to_degrees())),
        ),
        (
            "输出速度",
            "°/s",
            fb.as_ref().map(|f| format!("{:+.1}", f.speed.to_degrees())),
        ),
        (
            "输出力矩",
            "N·m",
            fb.as_ref().map(|f| format!("{:+.2}", f.torque)),
        ),
    ];
    let health = [
        ("壳体温度", fb.as_ref().map(|f| format!("{} °C", f.temp))),
        (
            "母线电压",
            fb.as_ref().map(|f| format!("{:.1} V", f.voltage)),
        ),
        (
            "设备故障",
            fb.as_ref().map(|f| {
                if f.error == 0 {
                    "无".into()
                } else {
                    "存在故障".into()
                }
            }),
        ),
        (
            "设备警告",
            fb.as_ref().map(|f| {
                if f.warning == 0 {
                    "无".into()
                } else {
                    "存在警告".into()
                }
            }),
        ),
    ];
    rsx! {
        style { "{STYLE}" }
        header { class:"mast",
            div { class:"brand", div { class:"brand-logo", role:"img", aria_label:"宇树科技", dangerous_inner_html: include_str!("../assets/unitree.svg") } strong { "S288" } span { "伺服控制台" } }
            div { class:"session",
                span { class:if connected() {"state online"} else {"state"}, if connected() {"串口已连接"} else {"串口未连接"} }
                button { id:"stop", class:"stop", disabled:!connected(), onclick:move |_| {
                    stop_output.call(());
                }, "■ 停止输出" kbd { "空格" } }
            }
        }
        main {
            section { class:"linkbar",
                label { r#for:"port", "串口" }
                select { id:"port", disabled:connected(), value:"{port()}", onchange:move |e|port.set(e.value()),
                    if ports().is_empty() { option { value:"", "未发现串口" } }
                    for p in ports() { option { value:"{p}", "{p}" } }
                }
                button { disabled:connected(), onclick:move |_| {
                    let found = scan_serial_ports();
                    if !found.contains(&port()) {port.set(found.first().cloned().unwrap_or_default());}
                    ports.set(found);
                }, "刷新" }
                button { id:"connect", class:if connected() {""} else {"primary"}, onclick:move |_| {
                    if connected() { link.disconnect(); connected.set(false); feedback.set(None); received.set(0); ids.set(vec![]); status.set("未应用控制".into()); }
                    else if port().is_empty() {show_error(message,"请接入串口适配器，刷新后选择串口。","connect");}
                    else if link.connect(&port()).is_ok() { connected.set(true); }
                    else {show_error(message,"无法打开串口。请检查适配器连接，并关闭占用该串口的程序。","port");}
                }, if connected() {"断开连接"} else {"连接串口"} }
                small { "6 Mbps · 8N1" }
            }
            div { class:"workspace",
                section { class:"panel",
                    div { class:"control-head",
                    h1 { "运动控制" }
                    div { class:"tabs", role:"group", aria_label:"控制逻辑",
                        for (label,next,active) in [("常规控制",basic(),mode()!=Mode::Mit),("MIT 控制",Mode::Mit,mode()==Mode::Mit)] {
                            button { aria_pressed:"{active}", onclick:move |_|switch_mode.call(next), "{label}" }
                        }
                    }
                    }
                    div { class:"setup",
                        label { r#for:"motor","电机 ID" }
                        select { id:"motor", value:"{id()}", onchange:move |e|if let Ok(v)=e.value().parse(){id.set(v)},
                            for n in 0u8..16 { option { value:"{n}", if ids.read().contains(&n) {"{n} · 已发现"} else {"{n}"} } }
                        }
                        if mode()!=Mode::Mit {
                            div { class:"segments", role:"group", aria_label:"常规控制模式",
                                for m in [Mode::Position,Mode::Velocity,Mode::Torque] {
                                    button { aria_pressed:"{mode()==m}", onclick:move |_|switch_mode.call(m), "{m.label()}" }
                                }
                            }
                        }
                    }
                    div { class:"fields",
                        for &i in mode().fields() {
                            div { class:"field", key:"{i}",
                                label { r#for:"value-{i}", "{FIELDS[i].0}" }
                                div { class:"edit",
                                    input { id:"value-{i}", r#type:"number", min:"{FIELDS[i].2}", max:"{FIELDS[i].3}", step:"{FIELDS[i].4}", value:"{draft.read()[i]}", oninput:move |e|draft.write()[i]=e.value() }
                                    span { "{FIELDS[i].1}" }
                                }
                                input { class:"slider", r#type:"range", aria_label:"{FIELDS[i].0} {FIELDS[i].1}", min:"{FIELDS[i].2}", max:"{FIELDS[i].3}", step:"{FIELDS[i].4}", value:"{draft.read()[i]}", oninput:move |e|draft.write()[i]=e.value() }
                                div { class:"limits", span {"{FIELDS[i].2}"} span {"{FIELDS[i].3}"} }
                            }
                        }
                    }
                    footer { button { class:"primary", id:"apply", disabled:!connected(), onclick:move |_| {
                        match setpoint(mode(), &draft.read()) {
                            Err(invalid) => {
                                let fields = invalid.iter().map(|&i|format!("{}（{} 至 {}）", FIELDS[i].0,FIELDS[i].2,FIELDS[i].3)).collect::<Vec<_>>().join("、");
                                show_error(message,format!("请输入有效数值：{fields}"),&format!("value-{}",invalid[0]));
                            }
                            Ok(command) => match serial.send_control(id(), command) {
                                Ok(()) => status.set(format!("ID {} · {}控制已应用",id(),mode().label())),
                                Err(()) => show_error(message,"指令未收到目标电机反馈。请检查电机 ID 和接线。","apply"),
                            }
                        }
                    }, "应用控制" } small { role:"status", "{status()}" } }
                }
                section { class:"panel feedback",
                    div { class:"heading", h2 { "实时反馈" } small { "{feedback_state}" } }
                    for (name,unit,value) in measurements {
                        div { class:"reading", span {"{name}"} div { strong {"{value.as_deref().unwrap_or(\"—\")}"} small {"{unit}"} } }
                    }
                    h3 { "设备状态" }
                    dl { for (name,value) in health { div {dt {"{name}"} dd {"{value.as_deref().unwrap_or(\"—\")}"} } } }
                    div { class:"frames", span {"有效反馈帧"} strong {"{received()}"} }
                }
            }
        }
        dialog { aria_labelledby:"error-heading",
            h2 { id:"error-heading","操作未完成" } p {"{message()}"} div { class:"dialog-actions", button {class:"primary",autofocus:true,onclick:move |_| { document::eval("document.querySelector('dialog').close()"); },"确认"} }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn control_modes_map_only_their_own_parameters() {
        let draft = ["45", "12", "2", "8", "0.4"].map(str::to_owned);
        let mit = setpoint(Mode::Mit, &draft).unwrap();
        assert_eq!(
            mit,
            Setpoint {
                position_deg: 45.,
                velocity_deg_s: 12.,
                torque_nm: 2.,
                kp: 8.,
                kd: 0.4
            }
        );
        assert_eq!(
            setpoint(Mode::Position, &draft).unwrap(),
            Setpoint {
                velocity_deg_s: 0.,
                torque_nm: 0.,
                ..mit
            }
        );
        assert_eq!(
            setpoint(Mode::Velocity, &draft).unwrap(),
            Setpoint {
                velocity_deg_s: 12.,
                kd: 0.4,
                ..Default::default()
            }
        );
        assert_eq!(
            setpoint(Mode::Torque, &draft).unwrap(),
            Setpoint {
                torque_nm: 2.,
                ..Default::default()
            }
        );
    }
}
