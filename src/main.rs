use std::net::UdpSocket;
use std::convert::TryInto;

struct Pose {
    x: f64,
    y: f64,
    z: f64,
    yaw: f64,
    pitch: f64,
    roll: f64,
}

fn udp_to_pose(port: u16) -> std::io::Result<()> {
    let socket = UdpSocket::bind(format!("0.0.0.0:{}", port))?;
    let mut buf = [0u8; 1024];

    'recv: loop {
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
        // Convert to Pose
        let pose = Pose {
            x: numbers[0],
            y: numbers[1],
            z: numbers[2],
            yaw: numbers[3],
            pitch: numbers[4],
            roll: numbers[5],
        };

        println!("x={:8.4}, y={:8.4}, z={:8.4}, roll={:8.4}, pitch={:8.4}, yaw={:8.4}", pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw);
    }
}

fn main() -> std::io::Result<()> {
    udp_to_pose(4242)
}