pub const CMD_LEN: usize = 20;
pub const FB_LEN: usize = 26;
const RATIO: f64 = 70070.0 / 243.0;

#[derive(Clone, Debug)]
pub struct Feedback {
    pub motor_id: u8,
    pub temp: i8,
    pub voltage: f32,
    pub torque: f32,
    pub speed: f32,
    pub position: f32,
    pub error: u32,
    pub warning: u8,
}

pub fn crc32_core(data: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFF_u32;
    for word in data.chunks_exact(4) {
        for byte in [word[3], word[2], word[1], word[0]] {
            crc ^= u32::from(byte) << 24;
            for _ in 0..8 {
                crc = if crc & 0x8000_0000 != 0 {
                    (crc << 1) ^ 0x04C1_1DB7
                } else {
                    crc << 1
                };
            }
        }
    }
    crc
}

fn clamp(value: f64, low: f64, high: f64) -> f64 {
    value.clamp(low, high)
}

pub fn build_control_packet(
    motor_id: u8,
    mode: u8,
    timeout: u8,
    torque: f64,
    speed: f64,
    position: f64,
    kp: f64,
    kd: f64,
) -> [u8; CMD_LEN] {
    let mode_byte = (motor_id & 0x0F) | ((mode & 0x07) << 4) | ((timeout & 1) << 7);
    let tor = clamp(torque, -36.908174, 36.908174) / RATIO * 256000.0;
    let spd = clamp(speed, -278.90143, 278.90143) * RATIO * 2.560 / std::f64::consts::PI / 2.0;
    let pos =
        clamp(position, -1428.018898, 1428.018898) * RATIO * 32768.0 / (2.0 * std::f64::consts::PI);
    let p = clamp(kp, 0.0, 2128.523254) / (RATIO * RATIO) * 1_280_000.0;
    let d = clamp(kd, 0.0, 21.285233) / (RATIO * RATIO) * 128_000_000.0;
    let mut packet = [0_u8; CMD_LEN];
    packet[0..2].copy_from_slice(&[0xFE, 0xEE]);
    packet[2] = mode_byte;
    packet[4..6].copy_from_slice(&(clamp(tor, -32768.0, 32767.0) as i16).to_le_bytes());
    packet[6..8].copy_from_slice(&(clamp(spd, -32768.0, 32767.0) as i16).to_le_bytes());
    packet[8..12]
        .copy_from_slice(&(clamp(pos, -2_147_483_648.0, 2_147_483_647.0) as i32).to_le_bytes());
    packet[12..14].copy_from_slice(&(clamp(p, -32768.0, 32767.0) as i16).to_le_bytes());
    packet[14..16].copy_from_slice(&(clamp(d, -32768.0, 32767.0) as i16).to_le_bytes());
    let crc = crc32_core(&packet[..16]);
    packet[16..20].copy_from_slice(&crc.to_le_bytes());
    packet
}

pub fn parse_feedback(frame: &[u8]) -> Option<Feedback> {
    if frame.len() != FB_LEN
        || frame[0..2] != [0xFC, 0xEE]
        || u32::from_le_bytes(frame[22..26].try_into().ok()?) != crc32_core(&frame[2..22])
    {
        return None;
    }
    let mode = frame[2];
    let temp = frame[3] as i8;
    let voltage = frame[5] as f32 / 2.0;
    let torque = f32::from(i16::from_le_bytes([frame[6], frame[7]])) / 256000.0 * RATIO as f32;
    let speed =
        f32::from(i16::from_le_bytes([frame[8], frame[9]])) / 2.56 * 2.0 * std::f32::consts::PI
            / RATIO as f32;
    let position =
        i32::from_le_bytes(frame[10..14].try_into().ok()?) as f32 / 32768.0 / RATIO as f32
            * 2.0
            * std::f32::consts::PI;
    let error = u32::from_le_bytes(frame[14..18].try_into().ok()?);
    let outpos = u16::from_le_bytes([frame[18], frame[19]]);
    Some(Feedback {
        motor_id: mode & 0x0F,
        temp,
        voltage,
        torque,
        speed,
        position,
        error,
        warning: ((outpos >> 13) & 7) as u8,
    })
}

pub fn take_feedback(stream: &mut Vec<u8>) -> Option<Feedback> {
    loop {
        let start = stream.windows(2).position(|pair| pair == [0xFC, 0xEE]);
        let Some(start) = start else {
            if stream.len() > FB_LEN {
                stream.drain(..stream.len() - 1);
            }
            return None;
        };
        if start > 0 {
            stream.drain(..start);
        }
        if stream.len() < FB_LEN {
            return None;
        }
        if let Some(feedback) = parse_feedback(&stream[..FB_LEN]) {
            stream.drain(..FB_LEN);
            return Some(feedback);
        }
        stream.drain(..1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn control_frame_matches_s288_crc() {
        let frame = build_control_packet(7, 1, 0, 0.0, 0.0, 0.0, 0.0, 0.0);
        assert_eq!(frame.len(), CMD_LEN);
        assert_eq!(
            frame,
            [
                0xfe, 0xee, 0x17, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x43, 0x4a, 0xbe, 0x82
            ]
        );
    }

    #[test]
    fn parser_ignores_command_echo_and_accepts_feedback() {
        let mut stream = build_control_packet(7, 1, 0, 0.0, 0.0, 0.0, 0.0, 0.0).to_vec();
        let mut frame = vec![
            0xfc, 0xee, 0x17, 42, 55, 48, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xd2, 0x04, 0, 0,
        ];
        let crc = crc32_core(&frame[2..]);
        frame.extend_from_slice(&crc.to_le_bytes());
        stream.extend_from_slice(&frame);
        let feedback = take_feedback(&mut stream).expect("valid FC EE feedback");
        assert_eq!(feedback.motor_id, 7);
        assert_eq!(feedback.temp, 42);
        assert!((feedback.voltage - 24.0).abs() < f32::EPSILON);
        assert!(stream.is_empty());
    }

    #[test]
    fn parser_rejects_bad_crc() {
        let mut frame = vec![
            0xfc, 0xee, 0x17, 42, 55, 48, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xd2, 0x04, 0, 0,
        ];
        let crc = crc32_core(&frame[2..]);
        frame.extend_from_slice(&crc.to_le_bytes());
        frame[10] ^= 0xff;
        assert!(parse_feedback(&frame).is_none());
    }
}
