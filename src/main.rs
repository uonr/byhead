use std::convert::TryInto;
use std::net::UdpSocket;

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

fn run(port: u16) -> std::io::Result<()> {
    use std::time::Instant;
    let socket = UdpSocket::bind(format!("0.0.0.0:{}", port))?;
    let mut buf = [0u8; 1024];
    let start = std::time::Instant::now();
    let (raw_tx, raw_rx) = std::sync::mpsc::sync_channel::<[f64; 6]>(1);
    let (sig_tx, sig_rx) = std::sync::mpsc::sync_channel::<Signal>(1);
    let _sig_thread = std::thread::spawn(move || {
        use niri_ipc::{Action, Request};
        let mut prev = std::time::Instant::now();
        let mut prev_signal = Signal::Nop;
        loop {
            let throttle_sec = 0.8;
            let ignore_sec = 0.4;
            let socket = niri_ipc::socket::Socket::connect().expect("Failed to connect to niri");
            let signal = sig_rx.recv().unwrap();
            let now = std::time::Instant::now();
            if now.duration_since(prev).as_secs_f64() < throttle_sec && signal == prev_signal {
                continue;
            }
            if now.duration_since(prev).as_secs_f64() < ignore_sec {
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
                        .send(Request::Action(Action::FocusWindowOrWorkspaceUp {  } ))
                        .unwrap();
                }
                Signal::Down => {
                    println!("Down");
                    let (_reply, _) = socket
                        .send(Request::Action(Action::FocusWindowOrWorkspaceDown {  } ))
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
            prev = now;
            prev_signal = signal;
        }
    });

    std::thread::spawn(move || {
        let mut history = std::collections::VecDeque::<PoseRecord>::with_capacity(1000);
        let mut prev_instant = Instant::now();
        let mut prev = Pose::default();
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
            if prev.x == 0.0 && prev.y == 0.0 && prev.z == 0.0 {
                prev = pose;
                prev_instant = now;
                continue;
            }

            let elapsed = now.duration_since(start).as_secs_f64();
            let delta = now.duration_since(prev_instant).as_secs_f64();
            if delta > 1.0 {
                println!("Delta too large: {}", delta);
                prev = pose;
                prev_instant = now;
                history.clear();
                continue;
            }

            let dx = pose.x - prev.x;
            let dy = pose.y - prev.y;
            let dz = pose.z - prev.z;
            let d_yaw = pose.yaw - prev.yaw;
            let d_pitch = pose.pitch - prev.pitch;
            let d_roll = pose.roll - prev.roll;
            let v_roll = d_roll.abs() / delta;
            let v_pitch = d_pitch.abs() / delta;
            let v_yaw = d_yaw.abs() / delta;
            let v_x = dx.abs() / delta;
            let v_y = dy.abs() / delta;
            let v_z = dz.abs() / delta;

            let record = PoseRecord {
                pose,
                instant: now,
                delta,
            };
            history.push_back(record);
            while history.len() > 1000 {
                history.pop_front();
            }
            if v_roll > 80.0 && roll.abs() > 9.0 {
                // println!(
                //     "[{:10.1}] v_roll={:6.1} roll={:6.1}",
                //     elapsed, v_roll, roll,
                // );
                if roll > 0.0 {
                    sig_tx.send(Signal::LeftColumn).unwrap();
                } else {
                    sig_tx.send(Signal::RightColumn).unwrap();
                }
            }
            if v_pitch > 80.0 && pitch.abs() > 9.0 {
                // println!(
                //     "[{:10.1}] v_pitch={:6.1} pitch={:6.1}",
                //     elapsed, v_pitch, pitch,
                // );
                if pitch > 0.0 {
                    sig_tx.send(Signal::Up).unwrap();
                } else {
                    sig_tx.send(Signal::Down).unwrap();
                }
            }
            if v_yaw > 120.0 && yaw.abs() > 16.0 {
                // println!(
                //     "[{:10.1}] v_yaw={:6.1} yaw={:6.1}",
                //     elapsed, v_yaw, yaw,
                // );
                if yaw > 0.0 {
                    sig_tx.send(Signal::RightMonitor).unwrap();
                } else {
                    sig_tx.send(Signal::LeftMonitor).unwrap();
                }
            }

            // println!(
            //     "[{:10.1}] x={:6.1} y={:6.1} z={:6.1} roll={:6.1} pitch={:6.1} yaw={:6.1}",
            //     elapsed, v_x, v_y, v_z, v_roll, v_pitch, v_yaw,
            // );

            prev = pose;
            prev_instant = now;
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
