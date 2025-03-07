use std::convert::TryInto;
use std::net::UdpSocket;
use std::time::Instant;

#[derive(Debug, Clone, Copy, PartialEq, Default)]
struct Pose {
    x: f64,
    y: f64,
    z: f64,
    yaw: f64,
    pitch: f64,
    roll: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct PoseRecord {
    pose: Pose,
    instant: std::time::Instant,
    delta: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Signal {
    LeftColumn,
    RightColumn,
    LeftMonitor,
    RightMonitor,
    Up,
    Down,
    Nop,
}

enum State {
    LeftYawing { start: Instant, end: Option<Instant> },
    RightYawing { start: Instant, end: Option<Instant> },
    Idle,
}

fn run(port: u16) -> std::io::Result<()> {
    let socket = UdpSocket::bind(format!("0.0.0.0:{}", port))?;
    let mut buf = [0u8; 1024];
    let start = std::time::Instant::now();
    let (raw_tx, raw_rx) = std::sync::mpsc::sync_channel::<[f64; 6]>(1);
    let (sig_tx, sig_rx) = std::sync::mpsc::sync_channel::<Signal>(1);
    let _sig_thread = std::thread::spawn(move || {
        use niri_ipc::{Action, Request};
        let mut prev_instant = std::time::Instant::now();
        loop {
            let socket = niri_ipc::socket::Socket::connect().expect("Failed to connect to niri");
            let signal = sig_rx.recv().unwrap();
            let now = std::time::Instant::now();
            let delta = now.duration_since(prev_instant).as_secs_f64();
            if delta < 0.35 {
                prev_instant = now;
                continue;
            }
            match signal {
                Signal::LeftColumn => {
                    println!("Left column");
                    let (_reply, _) = socket
                        .send(Request::Action(Action::FocusColumnLeft {}))
                        .unwrap();
                }
                Signal::RightColumn => {
                    println!("Right column");
                    let (_reply, _) = socket
                        .send(Request::Action(Action::FocusColumnRight {}))
                        .unwrap();
                }
                Signal::Up => {
                    println!("Up");
                    let (_reply, _) = socket
                        .send(Request::Action(Action::FocusWindowOrWorkspaceUp {}))
                        .unwrap();
                }
                Signal::Down => {
                    println!("Down");
                    let (_reply, _) = socket
                        .send(Request::Action(Action::FocusWindowOrWorkspaceDown {}))
                        .unwrap();
                }
                Signal::LeftMonitor => {
                    println!("Left screen");
                    let (_reply, _) = socket
                        .send(Request::Action(Action::FocusMonitorLeft {}))
                        .unwrap();
                }
                Signal::RightMonitor => {
                    println!("Right screen");
                    let (_reply, _) = socket
                        .send(Request::Action(Action::FocusMonitorRight {}))
                        .unwrap();
                }
                _ => {}
            }
            prev_instant = now;
        }
    });

    std::thread::spawn(move || {
        let mut state = State::Idle;
        let mut history = std::collections::VecDeque::<PoseRecord>::with_capacity(1000);
        loop {
            let now = std::time::Instant::now();
            let [x, y, z, yaw, pitch, roll] = raw_rx.recv().unwrap();
            let pose = Pose {
                x,
                y,
                z,
                yaw,
                pitch,
                roll,
            };

            let elapsed = now.duration_since(start).as_secs_f64();
            let Some(&prev_record) = history.back() else {
                history.push_back(PoseRecord {
                    pose,
                    instant: now,
                    delta: 0.0,
                });
                continue;
            };
            let prev: Pose = prev_record.pose;
            let prev_instant = prev_record.instant;
            let delta = now.duration_since(prev_instant).as_secs_f64();
            let record = PoseRecord {
                pose,
                instant: now,
                delta,
            };
            history.push_back(record);
            if history.len() < 16 {
                continue;
            }
            let Some(&prev_2_record) = history.get(history.len() - 3) else {
                continue;
            };
            let Some(&prev_3_record) = history.get(history.len() - 4) else {
                continue;
            };
            let prev_2 = prev_2_record.pose;
            let prev_3 = prev_3_record.pose;

            if delta > 1.0 {
                println!("Delta too large: {}", delta);
                history.clear();
                continue;
            }
            let d_t = now.duration_since(prev_2_record.instant).as_secs_f64();
            let v_x = (pose.x - prev_2.x) / d_t;
            let v_y = (pose.y - prev_2.y) / d_t;
            let v_z = (pose.z - prev_2.z) / d_t;
            let v_yaw = (pose.yaw - prev_2.yaw) / d_t;
            let v_pitch = (pose.pitch - prev_2.pitch) / d_t;
            let v_roll = (pose.roll - prev_2.roll) / d_t;
            let v = (v_x.powi(2) + v_y.powi(2) + v_z.powi(2)).sqrt();
            let v_rot = (v_yaw.powi(2) + v_pitch.powi(2) + v_roll.powi(2)).sqrt();

            let prev_d_t = prev_record
                .instant
                .duration_since(prev_3_record.instant)
                .as_secs_f64();
            let prev_v_x = (prev.x - prev_3.x) / prev_d_t;
            let prev_v_y = (prev.y - prev_3.y) / prev_d_t;
            let prev_v_z = (prev.z - prev_3.z) / prev_d_t;
            let prev_v_yaw = (prev.yaw - prev_3.yaw) / prev_d_t;
            let prev_v_pitch = (prev.pitch - prev_3.pitch) / prev_d_t;
            let prev_v_roll = (prev.roll - prev_3.roll) / prev_d_t;
            let prev_v = (prev_v_x.powi(2) + prev_v_y.powi(2) + prev_v_z.powi(2)).sqrt();
            let prev_v_rot =
                (prev_v_yaw.powi(2) + prev_v_pitch.powi(2) + prev_v_roll.powi(2)).sqrt();

            let accel_v = (v - prev_v) / d_t;
            let accel_v_rot = (v_rot - prev_v_rot) / d_t;

            if v_yaw.abs() > 60.0 {
                let right = v_yaw > 0.0;
                let arrow = if right { "→" } else { "←" };

                if right {
                    sig_tx.send(Signal::RightColumn).unwrap();
                } else {
                    sig_tx.send(Signal::LeftColumn).unwrap();
                }

                println!("[{:10.4}] {} v_yaw: {:6.2}", elapsed, arrow, v_yaw);
            } else if v_pitch > 45.0 || v_pitch < -72.0 {
                let up = v_pitch > 0.0;
                let arrow = if up { "↑" } else { "↓" };

                if up {
                    sig_tx.send(Signal::Up).unwrap();
                } else {
                    sig_tx.send(Signal::Down).unwrap();
                }

                println!("[{:10.4}] {} v_pitch: {:6.2}", elapsed, arrow, v_pitch);
            }
        }
    });

    'recv: loop {
        use std::sync::mpsc::TrySendError;
        let (number_of_bytes, _src) = socket.recv_from(&mut buf)?;
        if number_of_bytes != 48 {
            println!("Received {} bytes, expected 48", number_of_bytes);
            continue 'recv;
        }

        // https://github.com/opentrack/opentrack/issues/747
        let mut numbers = [0f64; 6];
        for i in 0..6 {
            let start = i * 8;
            let end = start + 8;
            let bytes: [u8; 8] = buf[start..end].try_into().unwrap();
            numbers[i] = f64::from_le_bytes(bytes);
        }
        if numbers.iter().any(|&x| x.is_nan()) {
            println!("Received NaN");
            continue 'recv;
        }
        match raw_tx.try_send(numbers) {
            Ok(_) => {}
            Err(TrySendError::Full(_)) => {
                println!("Dropped a frame");
            }
            Err(TrySendError::Disconnected(_)) => {
                println!("Receiver disconnected");
                return Ok(());
            }
        };
    }
}

fn main() -> std::io::Result<()> {
    let port = std::env::var("PORT")
        .expect("PORT is not set")
        .parse()
        .expect("Failed to parse PORT");
    run(port)
}
