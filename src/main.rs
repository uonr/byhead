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

impl std::ops::Add<Pose> for Pose {
    type Output = Pose;
    fn add(self, other: Pose) -> Pose {
        Pose {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
            yaw: self.yaw + other.yaw,
            pitch: self.pitch + other.pitch,
            roll: self.roll + other.roll,
        }
    }
}

impl std::ops::Sub<Pose> for Pose {
    type Output = Pose;
    fn sub(self, other: Pose) -> Pose {
        Pose {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
            yaw: self.yaw - other.yaw,
            pitch: self.pitch - other.pitch,
            roll: self.roll - other.roll,
        }
    }
}

impl std::ops::Mul<f64> for Pose {
    type Output = Pose;
    fn mul(self, other: f64) -> Pose {
        Pose {
            x: self.x * other,
            y: self.y * other,
            z: self.z * other,
            yaw: self.yaw * other,
            pitch: self.pitch * other,
            roll: self.roll * other,
        }
    }
}

impl std::ops::Div<f64> for Pose {
    type Output = Pose;
    fn div(self, other: f64) -> Pose {
        Pose {
            x: self.x / other,
            y: self.y / other,
            z: self.z / other,
            yaw: self.yaw / other,
            pitch: self.pitch / other,
            roll: self.roll / other,
        }
    }
}

impl Pose {
    fn displey(&self) -> String {
        format!(
            "x={:8.2} y={:8.2} z={:8.2} yaw={:8.2} pitch={:8.2} roll={:8.2}",
            self.x, self.y, self.z, self.yaw, self.pitch, self.roll
        )
    }
    fn diff(&self, other: Pose, delta: f64) -> Pose {
        (*self - other) / delta
    }

    fn yaw_arrow(&self) -> char {
        if self.yaw < 0.0 {
            '←'
        } else {
            '→'
        }
    }

    fn pitch_arrow(&self) -> char {
        if self.pitch > 0.0 {
            '↑'
        } else {
            '↓'
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct PoseRecord {
    pose: Pose,
    v: Pose,
    instant: std::time::Instant,
    delta: f64,
}

impl PoseRecord {
    fn diff(&self, other: PoseRecord) -> Pose {
        let delta = self.instant.duration_since(other.instant).as_secs_f64();
        self.pose.diff(other.pose, delta)
    }
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
    LeftYawing {
        start: Instant,
        end: Option<Instant>,
    },
    RightYawing {
        start: Instant,
        end: Option<Instant>,
    },
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
            if delta < 0.20 {
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
        let mut history = std::collections::VecDeque::<PoseRecord>::with_capacity(4096);
        loop {
            history.truncate(4000);
            let now = std::time::Instant::now();
            let [x, y, z, yaw, pitch, roll] = raw_rx.recv().expect("Failed to receive raw data");
            let pose = Pose {
                x,
                y,
                z,
                yaw,
                pitch,
                roll,
            };

            let elapsed = now.duration_since(start).as_secs_f64();
            let Some(&prev) = history.front() else {
                history.push_front(PoseRecord {
                    pose,
                    v: Pose::default(),
                    instant: now,
                    delta: 0.0,
                });
                continue;
            };
            let delta = now.duration_since(prev.instant).as_secs_f64();

            let record = if let Some(prev_2) = history.get(2) {
                let v = pose.diff(
                    prev_2.pose,
                    now.duration_since(prev_2.instant).as_secs_f64(),
                );
                PoseRecord {
                    pose,
                    v,
                    instant: now,
                    delta,
                }
            } else {
                let v = pose.diff(prev.pose, delta);
                PoseRecord {
                    pose,
                    v,
                    instant: now,
                    delta,
                }
            };
            history.push_front(record);
            if delta > 1.0 {
                println!("Delta too large: {}", delta);
                history.truncate(1);
                continue;
            }
            let history_len = history.len();
            if history_len < 16 {
                continue;
            }
            let yaw_threshold = 36.0;
            let pitch_threshold = 40.0;
            let idle_time = 500;
            let accel_threshold = 1000.0;
            let log_all = true;

            let from_idle = history
                .iter()
                .skip(1)
                .take_while(|x| now.duration_since(x.instant).as_millis() < idle_time)
                .all(|x| {
                    let v_yaw = x.v.yaw;
                    let v_pitch = x.v.pitch;
                    let same_direction = x.v.yaw.signum() == record.v.yaw.signum()
                        && v_pitch.signum() == record.v.pitch.signum();
                    (v_yaw.abs() < yaw_threshold && v_pitch.abs() < pitch_threshold)
                        || same_direction
                });

            let acc = history[0].v.diff(history[1].v, delta);
            if log_all && (elapsed.fract() * 100.0).floor() as i32 % 10 == 0 {
                let arrow = record.pose.pitch_arrow();
                let pitch = record.pose.pitch;
                let v_pitch = record.v.pitch;
                let acc_pitch = acc.pitch;
                println!(
                    "[{elapsed:10.3}] {arrow} pitch={pitch:8.4} v_pitch={v_pitch:8.4} acc_patch={acc_pitch:12.4}",
                );
            }
            if record.v.yaw.abs() >= yaw_threshold
                && acc.yaw.abs() > accel_threshold
                && record.v.yaw.abs() > record.v.pitch.abs()
            {
                if !from_idle {
                    println!("[YAW] NOT IDLE");
                } else {
                    let acc_yaw = acc.yaw;
                    let v_yaw = record.v.yaw;
                    let arrow = record.v.yaw_arrow().to_string().repeat(8);
                    println!("{arrow} {v_yaw:12.4} acc_yaw={acc_yaw:12.4}",);
                    sig_tx
                        .send(if record.v.yaw < 0.0 {
                            Signal::LeftColumn
                        } else {
                            Signal::RightColumn
                        })
                        .unwrap();
                }
            } else if record.v.pitch.abs() > pitch_threshold && acc.pitch.abs() > accel_threshold {
                if !from_idle {
                    println!("[PITCH] NOT IDLE");
                } else {
                    println!(
                        "{} {from_idle}",
                        record.v.pitch_arrow().to_string().repeat(8),
                    );
                    sig_tx
                        .send(if record.v.pitch < 0.0 {
                            Signal::Down
                        } else {
                            Signal::Up
                        })
                        .unwrap();
                }
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
