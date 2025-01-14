use serialport::{ClearBuffer, DataBits, Parity, SerialPort, StopBits};
use std::collections::HashMap;
use std::io::{Read, Write};
use std::thread;
use std::time::Duration;

use crate::types::*;

fn get_common_params() -> HashMap<&'static str, Vec<u8>> {
    HashMap::from([
        ("BATTERY", vec![0x01, 100]),
        ("PERFORM", vec![0x03, 0]),
        ("CALIBRATION", vec![0x04, 0]),
        ("UPGRADE", vec![0x05, 0]),
        ("SET_ORIGIN", vec![0x06, 1]),
        ("FIRMWARE_VERSION", vec![0x07]),
        ("GAIT_TYPE", vec![0x09, 0x00]),
        ("BT_NAME", vec![0x13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]),
        ("UNLOAD_MOTOR", vec![0x20, 0]),
        ("LOAD_MOTOR", vec![0x20, 0]),
        ("VX", vec![0x30, 128]),
        ("VY", vec![0x31, 128]),
        ("VYAW", vec![0x32, 128]),
        ("TRANSLATION", vec![0x33, 0, 0, 0]),
        ("ATTITUDE", vec![0x36, 0, 0, 0]),
        ("PERIODIC_ROT", vec![0x39, 0, 0, 0]),
        ("MarkTime", vec![0x3C, 0]),
        ("MOVE_MODE", vec![0x3D, 0]),
        ("ACTION", vec![0x3E, 0]),
        ("MOVE_TO", vec![0x3F, 0, 0]),
        ("PERIODIC_TRAN", vec![0x80, 0, 0, 0]),
        (
            "MOTOR_ANGLE",
            vec![
                0x50, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128,
            ],
        ),
        ("MOTOR_SPEED", vec![0x5C, 1]),
        ("MOVE_TO_MID", vec![0x5F, 1]),
        ("LEG_POS", vec![0x40, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]),
        ("IMU", vec![0x61, 0]),
        ("ROLL", vec![0x62, 0]),
        ("PITCH", vec![0x63, 0]),
        ("TEACH_RECORD", vec![0x21, 0]),
        ("TEACH_PLAY", vec![0x22, 0]),
        ("TEACH_ARM_RECORD", vec![0x23, 0]),
        ("TEACH_ARM_PLAY", vec![0x24, 0]),
        ("YAW", vec![0x64, 0]),
        ("CLAW", vec![0x71, 0]),
        ("ARM_MODE", vec![0x72, 0]),
        ("ARM_X", vec![0x73, 0]),
        ("ARM_Z", vec![0x74, 0]),
        ("ARM_SPEED", vec![0x75, 0]),
        ("ARM_THETA", vec![0x76, 0]),
        ("ARM_R", vec![0x77, 0]),
        ("OUTPUT_ANALOG", vec![0x90, 0]),
        ("OUTPUT_DIGITAL", vec![0x91, 0]),
        ("LED_COLOR", vec![0x69, 0, 0, 0]),
    ])
}

fn copysign(x: f64, y: f64) -> f64 {
    if y.is_sign_positive() {
        x.abs()
    } else {
        -x.abs()
    }
}

fn conver2u8(data: f64, limit: Vec<f64>, min_value: Option<u8>) -> u8 {
    //将实际参数转化为0到255的单字节数据
    //Convert the actual parameters to single byte data from 0 to 255
    let min_value = min_value.unwrap_or(0);

    let max_value: u8 = 0xff;

    let limit = if limit.len() == 1 {
        vec![-limit[0], limit[0]]
    } else {
        limit
    };

    if data >= limit[1] {
        max_value
    } else if data <= limit[0] {
        min_value
    } else {
        ((255.0 / (limit[1] - limit[0])) * (data - limit[0])) as u8
    }
}

fn conver2float(data: u8, limit: impl Into<Vec<f64>>) -> f64 {
    let limit: Vec<f64> = limit.into();
    if limit.len() == 1 {
        (data as f64 - 128.0) / 255.0 * limit[0]
    } else {
        (data as f64 / 255.0) * (limit[1] - limit[0]) + limit[0]
    }
}

fn byte2float(rawdata: &[u8]) -> f32 {
    let mut a = [0u8; 4];
    a[0] = rawdata[3];
    a[1] = rawdata[2];
    a[2] = rawdata[1];
    a[3] = rawdata[0];
    f32::from_le_bytes(a)
}

fn byte2short(rawdata: &[u8]) -> i16 {
    let mut a = [0u8; 2];
    a[0] = rawdata[1];
    a[1] = rawdata[0];
    i16::from_be_bytes(a)
}

fn round_to_2_decimal_places(num: f64) -> f64 {
    (num * 100.0).round() / 100.0
}

fn get_device(version: &str) -> XGODevice {
    if version.eq("xgomini") {
        XGODevice {
            translation_limit: TranslationLimit {
                x_limit: 35.0,
                y_limit: 19.5,
                z_limit: [75.0, 120.0],
            },
            attitude_limit: AttitudeLimit {
                roll_limit: 20.0,
                pitch_limit: 22.0,
                yaw_limit: 16.0,
            },
            leg_limit: LegLimit {
                x_limit: 35.0,
                y_limit: 18.0,
                z_limit: [75.0, 115.0],
            },
            motor_limit: MotorLimit {
                limit: [
                    [-73.0, 57.0],
                    [-66.0, 93.0],
                    [-31.0, 31.0],
                    [-65.0, 65.0],
                    [-85.0, 50.0],
                    [-75.0, 90.0],
                ],
            },
            period_limit: PeriodLimit {
                limit: [[1.5, 8.0]],
            },
            mark_time_limit: MarkTimeLimit {
                limit: [10.0, 35.0],
            },
            vx_limit: VXLimit { limit: 25.0 },
            vy_limit: VYLimit { limit: 18.0 },
            vyaw_limit: VYAWLimit { limit: 100.0 },
            arm_limit: ARMLimit {
                limit: [[-80.0, 155.0], [-95.0, 155.0], [70.0, 270.0], [80.0, 140.0]],
            },
            action_time: ActionTime {
                action_time: HashMap::from([
                    (1, 3),
                    (2, 3),
                    (3, 5),
                    (4, 5),
                    (5, 4),
                    (6, 4),
                    (7, 4),
                    (8, 4),
                    (9, 4),
                    (10, 7),
                    (11, 7),
                    (12, 5),
                    (13, 7),
                    (14, 10),
                    (15, 6),
                    (16, 6),
                    (17, 4),
                    (18, 6),
                    (19, 10),
                    (20, 9),
                    (21, 8),
                    (22, 7),
                    (23, 6),
                    (24, 7),
                    (128, 10),
                    (129, 10),
                    (130, 10),
                    (255, 1),
                ]),
            },
        }
    } else if version.eq("xgolite") {
        XGODevice {
            translation_limit: TranslationLimit {
                x_limit: 35.0,
                y_limit: 19.5,
                z_limit: [60.0, 110.0],
            },
            attitude_limit: AttitudeLimit {
                roll_limit: 20.0,
                pitch_limit: 10.0,
                yaw_limit: 12.0,
            },
            leg_limit: LegLimit {
                x_limit: 35.0,
                y_limit: 18.0,
                z_limit: [60.0, 110.0],
            },
            motor_limit: MotorLimit {
                limit: [
                    [-70.0, 50.0],
                    [-70.0, 90.0],
                    [-30.0, 30.0],
                    [-65.0, 65.0],
                    [-115.0, 70.0],
                    [-85.0, 100.0],
                ],
            },
            period_limit: PeriodLimit {
                limit: [[1.5, 8.0]],
            },
            mark_time_limit: MarkTimeLimit {
                limit: [10.0, 25.0],
            },
            vx_limit: VXLimit { limit: 25.0 },
            vy_limit: VYLimit { limit: 18.0 },
            vyaw_limit: VYAWLimit { limit: 100.0 },
            arm_limit: ARMLimit {
                limit: [[-80.0, 155.0], [-95.0, 155.0], [70.0, 270.0], [80.0, 140.0]],
            },
            action_time: ActionTime {
                action_time: HashMap::from([
                    (1, 3),
                    (2, 3),
                    (3, 5),
                    (4, 5),
                    (5, 4),
                    (6, 4),
                    (7, 4),
                    (8, 4),
                    (9, 4),
                    (10, 7),
                    (11, 7),
                    (12, 5),
                    (13, 7),
                    (14, 10),
                    (15, 6),
                    (16, 6),
                    (17, 4),
                    (18, 6),
                    (19, 10),
                    (20, 9),
                    (21, 8),
                    (22, 7),
                    (23, 6),
                    (24, 7),
                    (128, 10),
                    (129, 10),
                    (130, 10),
                    (255, 1),
                ]),
            },
        }
    } else if version.eq("xgorider") {
        XGODevice {
            translation_limit: TranslationLimit {
                x_limit: 1.0,
                y_limit: 1.0,
                z_limit: [60.0, 120.0],
            },
            attitude_limit: AttitudeLimit {
                roll_limit: 17.0,
                pitch_limit: 1.0,
                yaw_limit: 1.0,
            },
            leg_limit: LegLimit {
                x_limit: 1.0,
                y_limit: 1.0,
                z_limit: [60.0, 120.0],
            },
            motor_limit: MotorLimit {
                limit: [
                    [-1.0, 1.0],
                    [-1.0, 1.0],
                    [-1.0, 1.0],
                    [-1.0, 1.0],
                    [-1.0, 1.0],
                    [-1.0, 1.0],
                ],
            },
            period_limit: PeriodLimit {
                limit: [[1.0, 2.0]],
            },
            mark_time_limit: MarkTimeLimit { limit: [-1.0, 1.0] },
            vx_limit: VXLimit { limit: 1.5 },
            vy_limit: VYLimit { limit: 1.0 },
            vyaw_limit: VYAWLimit { limit: 360.0 },
            arm_limit: ARMLimit {
                limit: [[-1.0, 1.0], [-1.0, 1.0], [-1.0, 1.0], [-1.0, 1.0]],
            },
            action_time: ActionTime {
                action_time: HashMap::from([
                    (1, 3),
                    (2, 3),
                    (3, 5),
                    (4, 5),
                    (5, 4),
                    (6, 4),
                    (7, 4),
                    (8, 4),
                    (9, 4),
                    (10, 7),
                    (11, 7),
                    (12, 5),
                    (13, 7),
                    (14, 10),
                    (15, 6),
                    (16, 6),
                    (17, 4),
                    (18, 6),
                    (19, 10),
                    (20, 9),
                    (21, 8),
                    (22, 7),
                    (23, 6),
                    (24, 7),
                    (128, 10),
                    (129, 10),
                    (130, 10),
                    (255, 1),
                ]),
            },
        }
    } else {
        XGODevice {
            translation_limit: TranslationLimit {
                x_limit: 0.0,
                y_limit: 0.0,
                z_limit: [0.0, 0.0],
            },
            attitude_limit: AttitudeLimit {
                roll_limit: 0.0,
                pitch_limit: 0.0,
                yaw_limit: 0.0,
            },
            leg_limit: LegLimit {
                x_limit: 0.0,
                y_limit: 0.0,
                z_limit: [0.0, 0.0],
            },
            motor_limit: MotorLimit {
                limit: [
                    [0.0, 0.0],
                    [0.0, 0.0],
                    [0.0, 0.0],
                    [0.0, 0.0],
                    [0.0, 0.0],
                    [0.0, 0.0],
                ],
            },
            period_limit: PeriodLimit {
                limit: [[0.0, 0.0]],
            },
            mark_time_limit: MarkTimeLimit { limit: [0.0, 0.0] },
            vx_limit: VXLimit { limit: 0.0 },
            vy_limit: VYLimit { limit: 0.0 },
            vyaw_limit: VYAWLimit { limit: 0.0 },
            arm_limit: ARMLimit {
                limit: [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]],
            },
            action_time: ActionTime {
                action_time: Default::default(),
            },
        }
    }
}

#[allow(dead_code)]
pub struct XGO {
    verbose: bool,
    port: Box<dyn SerialPort>,
    port_name: String,
    device: XGODevice,
    common_params: HashMap<&'static str, Vec<u8>>,
    rx_flag: u8,
    rx_count: usize,
    rx_addr: u8,
    rx_len: usize,
    rx_data: Vec<u8>,
    rx_type: u8,
    version: Option<String>,
    mintime: f64,
    init_yaw: f64,
}

impl XGO {
    pub fn new(port_name: String, baud: u32, verbose: bool) -> Self {
        let mut port = serialport::new(&port_name, baud)
            .timeout(Duration::from_millis(500))
            .data_bits(DataBits::Eight)
            .parity(Parity::None)
            .stop_bits(StopBits::One)
            .open()
            .expect(&format!("failed to open port {:?}", &port_name));

        port.flush().unwrap();
        thread::sleep(Duration::from_millis(250));

        let device = get_device("NA");

        let mut xgo = XGO {
            verbose,
            port,
            port_name,
            device,
            common_params: get_common_params(),
            rx_flag: 0,
            rx_count: 0,
            rx_addr: 0,
            rx_len: 0,
            rx_data: vec![0; 50],
            rx_type: 0,
            version: None,
            mintime: 0.65,
            init_yaw: 0.0,
        };

        let version = xgo.read_firmware();
        if version.eq("M") {
            xgo.device = get_device("xgomini");
        } else if version.eq("L") {
            xgo.device = get_device("xgolite");
        } else if version.eq("R") {
            xgo.device = get_device("xgorider");
        } else {
            xgo.device = get_device("NA");
        }
        xgo.version = Some(version);

        xgo.reset();
        xgo.init_yaw = xgo.read_yaw();
        thread::sleep(Duration::from_secs(1));
        xgo
    }

    fn set_param(&mut self, key: &str, index: usize, new_value: u8) {
        if let Some(value) = self.common_params.get_mut(key) {
            value[index] = new_value;
        }
    }

    fn __flush_input(&mut self) {
        self.port.clear(ClearBuffer::Input).unwrap();
    }

    fn __flush_output(&mut self) {
        self.port.clear(ClearBuffer::Output).unwrap();
    }

    fn __send(&mut self, key: &str, index: Option<u8>, len: Option<u8>) {
        let index = index.unwrap_or(1);
        let len = len.unwrap_or(1);

        let mode = 0x01;
        let order = self.common_params[key][0] + index - 1;
        let mut value = vec![];
        let mut value_sum = 0;
        for i in index..index + len {
            value.push(self.common_params[key][i as usize]);
            value_sum += self.common_params[key][i as usize];
        }
        let sum_data = (len as u16 + 0x08u16 + mode as u16 + order as u16 + value_sum as u16) % 256;
        let sum_data = (255 - sum_data) as u8;
        let mut tx = vec![0x55, 0x00, len + 0x08, mode, order];
        tx.extend(value);
        tx.extend(vec![sum_data, 0x00, 0xAA]);
        self.port.write(&tx).unwrap();
        if self.verbose {
            println!("tx_data: {:?}", tx);
        }
    }

    fn __read(&mut self, addr: u8, read_len: Option<u8>) {
        let read_len = read_len.unwrap_or(1);

        let mode = 0x02;
        let sum_data = (0x09u16 + mode as u16 + addr as u16 + read_len as u16) % 256;
        let sum_data = (255 - sum_data) as u8;
        let tx = vec![0x55, 0x00, 0x09, mode, addr, read_len, sum_data, 0x00, 0xAA];
        self.__flush_input();
        self.port.write(&tx).unwrap();
        if self.verbose {
            println!("tx_data: {:?}", tx);
        }
    }

    fn __change_baud(&mut self, baud: u32) {
        self.port.flush().unwrap();
        self.port.set_baud_rate(baud).unwrap();
    }

    pub fn stop(&mut self) {
        self.move_x(0f32, None);
        self.move_y(0f32, None);
        self.mark_time(0);
        self.turn(0, None);
    }

    pub fn move_direction(&mut self, direction: &str, step: f32) {
        match direction.to_lowercase().as_str() {
            "x" => self.move_x(step, None),
            "y" => self.move_y(step, None),
            _ => println!("ERROR! Invalid direction!"),
        }
    }

    fn move_x(&mut self, step: f32, runtime: Option<f64>) {
        let runtime = runtime.unwrap_or(0f64);
        self.set_param(
            "VX",
            1,
            conver2u8(step as f64, vec![self.device.vx_limit.limit], None),
        );
        self.__send("VX", None, None);

        if runtime > 0.0 {
            thread::sleep(Duration::from_secs_f64(runtime));
            self.set_param(
                "VX",
                1,
                conver2u8(0f64, vec![self.device.vx_limit.limit], None),
            );
            self.__send("VX", None, None);
        }
    }

    fn move_y(&mut self, step: f32, runtime: Option<f64>) {
        let runtime = Some(runtime).unwrap_or(Some(0f64)).unwrap();

        self.set_param(
            "VY",
            1,
            conver2u8(step as f64, vec![self.device.vy_limit.limit], None),
        );
        self.__send("VY", None, None);

        if runtime > 0.0 {
            thread::sleep(Duration::from_secs_f64(runtime));
            self.set_param(
                "VY",
                1,
                conver2u8(0f64, vec![self.device.vy_limit.limit], None),
            );
            self.__send("VY", None, None);
        }
    }

    pub fn turn(&mut self, step: i32, runtime: Option<f64>) {
        let runtime = runtime.unwrap_or(0f64);

        self.set_param(
            "VYAW",
            1,
            conver2u8(step as f64, vec![self.device.vyaw_limit.limit], None),
        );
        self.__send("VYAW", None, None);

        if runtime > 0.0 {
            thread::sleep(Duration::from_secs_f64(runtime));
            self.set_param(
                "VYAW",
                1,
                conver2u8(0f64, vec![self.device.vyaw_limit.limit], None),
            );
            self.__send("VYAW", None, None);
        }
    }

    fn move_by(&mut self, distance: f64, vx: i32, vy: i32, k: f64, mintime: f64) {
        let runtime = distance.abs() * k + mintime;
        self.move_x(copysign(vx as f64, distance) as f32, None);
        self.move_y(copysign(vy as f64, distance) as f32, None);
        thread::sleep(Duration::from_secs_f64(runtime));
        self.move_x(0f32, None);
        self.move_y(0f32, None);
        thread::sleep(Duration::from_secs_f64(0.2));
    }

    pub fn move_x_by(
        &mut self,
        distance: f64,
        vx: Option<i32>,
        k: Option<f64>,
        mintime: Option<f64>,
    ) {
        self.move_by(
            distance,
            vx.unwrap_or(18),
            0,
            k.unwrap_or(0.035),
            mintime.unwrap_or(0.55),
        );
    }

    pub fn move_y_by(
        &mut self,
        distance: f64,
        vy: Option<i32>,
        k: Option<f64>,
        mintime: Option<f64>,
    ) {
        self.move_by(
            distance,
            0,
            vy.unwrap_or(18),
            k.unwrap_or(0.0373),
            mintime.unwrap_or(0.5),
        );
    }

    pub fn turn_by(&mut self, theta: f64, mintime: f64, vyaw: Option<i32>, k: Option<f64>) {
        let runtime = theta.abs() * k.unwrap_or(0.08) + mintime;
        self.turn(copysign(vyaw.unwrap_or(16) as f64, theta) as i32, None);
        thread::sleep(Duration::from_secs_f64(runtime));
        self.turn(0, None);
    }

    pub fn turn_to(&mut self, theta: f64, vyaw: Option<i32>, emax: Option<i32>) {
        let mut cur_yaw = self.read_yaw();
        let des_yaw = self.init_yaw + theta;

        while (des_yaw - cur_yaw).abs() >= emax.unwrap_or(10) as f64 {
            self.turn(
                copysign(vyaw.unwrap() as f64, des_yaw - cur_yaw) as i32,
                None,
            );
            cur_yaw = self.read_yaw();
        }

        self.turn(0, None);
        thread::sleep(Duration::from_secs_f64(0.2));
    }

    /// Move forward
    pub fn move_forward(&mut self, step: f32) {
        self.move_x(step.abs(), None);
    }

    /// Move backward
    pub fn move_backward(&mut self, step: f32) {
        self.move_x(-step.abs(), None);
    }

    /// Move left
    pub fn move_left(&mut self, step: f32) {
        self.move_y(step.abs(), None);
    }

    /// Move right
    pub fn move_right(&mut self, step: f32) {
        self.move_y(-step.abs(), None);
    }

    /// Turn left
    pub fn turn_left(&mut self, step: i32) {
        self.turn(step.abs(), None);
    }

    /// Turn right
    pub fn turn_right(&mut self, step: i32) {
        self.turn(-step.abs(), None);
    }

    fn __translation(&mut self, direction: &str, period: f32) {
        let index = Vec::from(["x", "y", "z"])
            .iter()
            .position(|&x| x == direction.to_lowercase().as_str());

        match index {
            None => {
                println!("ERROR! Direction must be 'x', 'y' or 'z'");
                return;
            }
            Some(index) => {
                match index {
                    0 => self.set_param(
                        "TRANSLATION",
                        index,
                        conver2u8(
                            period as f64,
                            vec![self.device.translation_limit.x_limit],
                            None,
                        ),
                    ),
                    1 => self.set_param(
                        "TRANSLATION",
                        index,
                        conver2u8(
                            period as f64,
                            vec![self.device.translation_limit.y_limit],
                            None,
                        ),
                    ),
                    2 => self.set_param(
                        "TRANSLATION",
                        index,
                        conver2u8(
                            period as f64,
                            vec![
                                self.device.translation_limit.z_limit[0],
                                self.device.translation_limit.z_limit[1],
                            ],
                            None,
                        ),
                    ),
                    _ => return,
                }

                self.__send("TRANSLATION", Some(index as u8), None);
            }
        }
    }

    /// Keep the robot's feet stationary and the body makes three-axis translation
    /// 使机器狗足端不动，身体进行三轴平动
    pub fn translation(&mut self, direction: &str, period: f32) {
        self.__translation(direction, period);
    }

    /// Keep the robot's feet stationary and the body makes three-axis rotation (Multiple Directions)
    /// 使机器狗足端不动，身体进行三轴转动 (多个方向)
    pub fn translation_all(&mut self, directions: Vec<&str>, periods: Vec<f32>) {
        if directions.len() != periods.len() {
            println!("ERROR! The length of direction and periods does not match!");
            return;
        }

        for i in 0..directions.len() {
            self.__translation(directions[i], periods[i]);
        }
    }

    fn __attitude(&mut self, direction: &str, period: f32) {
        let index = Vec::from(["r", "p", "y"])
            .iter()
            .position(|&x| x == direction.to_lowercase().as_str());

        match index {
            None => {
                println!("ERROR! Direction must be 'r', 'p' or 'y'");
                return;
            }
            Some(index) => {
                match index {
                    0 => self.set_param(
                        "ATTITUDE",
                        index,
                        conver2u8(
                            period as f64,
                            vec![self.device.attitude_limit.roll_limit],
                            None,
                        ),
                    ),
                    1 => self.set_param(
                        "ATTITUDE",
                        index,
                        conver2u8(
                            period as f64,
                            vec![self.device.attitude_limit.pitch_limit],
                            None,
                        ),
                    ),
                    2 => self.set_param(
                        "ATTITUDE",
                        index,
                        conver2u8(
                            period as f64,
                            vec![self.device.attitude_limit.yaw_limit],
                            None,
                        ),
                    ),
                    _ => return,
                }

                self.__send("ATTITUDE", Some(index as u8), None);
            }
        }
    }

    /// Keep the robot's feet stationary and the body makes three-axis rotation
    /// 使机器狗足端不动，身体进行三轴转动
    pub fn attitude(&mut self, direction: &str, period: f32) {
        self.__attitude(direction, period);
    }

    /// Keep the robot's feet stationary and the body makes three-axis rotation (Multiple Directions)
    /// 使机器狗足端不动，身体进行三轴转动 (多个方向)
    pub fn attitude_all(&mut self, directions: Vec<&str>, periods: Vec<f32>) {
        if directions.len() != periods.len() {
            println!("ERROR! The length of direction and periods does not match!");
            return;
        }

        for i in 0..directions.len() {
            self.__attitude(directions[i], periods[i]);
        }
    }

    /// Make the robot do the specified preset action
    /// 使机器狗狗指定的预设动作
    pub fn action(&mut self, action_id: i32, wait: Option<bool>) {
        let wait = wait.unwrap_or(false);

        if action_id == 0 || action_id > 255 {
            println!("ERROR! Illegal Action ID!");
        }

        self.set_param("ACTION", 1, action_id as u8);
        self.__send("ACTION", None, None);

        if wait {
            if self.device.action_time.action_time.contains_key(&action_id) {
                thread::sleep(Duration::from_secs_f64(
                    self.device.action_time.action_time[&action_id].into(),
                ));
            }
        }
    }

    /// The robot dog stops moving and all parameters return to the initial state
    /// 机器狗停止运动，所有参数恢复到初始状态
    pub fn reset(&mut self) {
        self.action(255, None);
        thread::sleep(Duration::from_secs_f64(1.0));
    }

    /// Control the three-axis movement of a single leg
    /// 控制机器狗的单腿的三轴移动
    pub fn leg(&mut self, leg_id: u8, data: Vec<i32>) {
        if leg_id < 1 || leg_id > 4 {
            println!("Error! Illegal Index!");
            return;
        }
        if data.len() != 3 {
            println!("Error! Illegal Value!");
            return;
        }

        let mut value = [0u8; 3];
        value[0] = conver2u8(data[0] as f64, vec![self.device.leg_limit.x_limit], None);
        value[1] = conver2u8(data[1] as f64, vec![self.device.leg_limit.y_limit], None);
        value[2] = conver2u8(
            data[2] as f64,
            Vec::from(&mut self.device.leg_limit.z_limit),
            None,
        );

        for i in 0..3 {
            let index = 3 * (leg_id - 1) as usize + i + 1;
            self.set_param("LEG_POS", index, value[i]);
            self.__send("LEG_POS", Some(index as u8), None);
        }
    }

    fn __motor(&mut self, index: usize, data: i32) {
        if index < 13 {
            self.set_param(
                "MOTOR_ANGLE",
                index,
                conver2u8(
                    data as f64,
                    Vec::from(self.device.motor_limit.limit[(index - 1) % 3]),
                    None,
                ),
            );
        } else if index == 13 {
            // self.claw
            return;
        } else {
            self.set_param(
                "MOTOR_ANGLE",
                index,
                conver2u8(
                    data as f64,
                    Vec::from(self.device.motor_limit.limit[index - 9]),
                    None,
                ),
            );
        }

        self.__send("MOTOR_ANGLE", Some(index as u8), None);
    }

    /// Control the rotation of a single steering gear of the robot
    /// 控制机器狗单个舵机转动
    pub fn motor(&mut self, motor_id: usize, data: i32) {
        let all_motor_ids = [11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, 43, 51, 52, 53];
        let index = Vec::from(all_motor_ids).iter().position(|&x| x == motor_id);

        match index {
            None => {
                println!("Error! Illegal motor_id!");
            }
            Some(index) => {
                self.__motor(index, data);
            }
        }
    }

    /// Control the rotation of multiple steering gears of the robot
    /// 控制机器狗多个舵机转动
    pub fn motors(&mut self, motor_ids: Vec<usize>, data: Vec<i32>) {
        let all_motor_ids = [11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, 43, 51, 52, 53];

        for i in 0..motor_ids.len() {
            let index = Vec::from(all_motor_ids)
                .iter()
                .position(|&x| x == motor_ids[i]);
            match index {
                None => {
                    println!("Error! Illegal motor_id!");
                }
                Some(index) => {
                    self.__motor(index, data[i]);
                }
            }
        }
    }

    /// Unload Motor
    /// 卸载舵机
    pub fn unload_motor(&mut self, leg_id: u8) {
        if !Vec::from([1, 2, 3, 4, 5]).contains(&leg_id) {
            println!("ERROR! leg_id must be 1, 2, 3 ,4 or 5");
            return;
        }

        self.set_param("UNLOAD_MOTOR", 1, 0x10 + leg_id);
        self.__send("UNLOAD_MOTOR", None, None);
    }

    /// Unload All Motors
    /// 卸载所有舵机
    pub fn unload_all_motors(&mut self) {
        self.set_param("UNLOAD_MOTOR", 1, 0x01);
        self.__send("UNLOAD_MOTOR", None, None);
    }

    /// Load Motor
    /// 载入舵机
    pub fn load_motor(&mut self, leg_id: u8) {
        if !Vec::from([1, 2, 3, 4, 5]).contains(&leg_id) {
            println!("ERROR! leg_id must be 1, 2, 3 ,4 or 5");
            return;
        }

        self.set_param("LOAD_MOTOR", 1, 0x20 + leg_id);
        self.__send("LOAD_MOTOR", None, None);
    }

    /// Load All Motors
    /// 载入所有舵机
    pub fn load_all_motors(&mut self) {
        self.set_param("LOAD_MOTOR", 1, 0x00);
        self.__send("LOAD_MOTOR", None, None);
    }

    fn __periodic_rotate(&mut self, direction: &str, period: f32) {
        let index = Vec::from(["r", "p", "y"])
            .iter()
            .position(|&x| x == direction.to_lowercase().as_str());

        match index {
            None => {
                println!("ERROR! Direction must be 'r', 'p' or 'y'");
                return;
            }
            Some(index) => {
                if period == 0f32 {
                    self.set_param("PERIODIC_ROT", index, 0);
                } else {
                    self.set_param(
                        "PERIODIC_ROT",
                        index,
                        conver2u8(
                            period as f64,
                            Vec::from(self.device.period_limit.limit[0]),
                            Some(1),
                        ),
                    );
                }
                self.__send("PERIODIC_ROT", Some(index as u8), None);
            }
        }
    }

    /// Make the robot rotate periodically
    /// 使机器狗周期性转动
    pub fn periodic_rotate(&mut self, direction: &str, period: f32) {
        self.__periodic_rotate(direction, period);
    }

    /// Make the robot rotate periodically (Multiple Directions)
    /// 使机器狗周期性转动 (多个方向)
    pub fn periodic_rotate_all(&mut self, directions: Vec<&str>, periods: Vec<f32>) {
        if directions.len() != periods.len() {
            println!("ERROR! The length of direction and periods does not match!");
            return;
        }
        for i in 0..directions.len() {
            self.__periodic_rotate(directions[i], periods[i]);
        }
    }

    fn __periodic_tran(&mut self, direction: &str, period: f32) {
        let index = Vec::from(["x", "y", "z"])
            .iter()
            .position(|&x| x == direction.to_lowercase().as_str());

        match index {
            None => {
                println!("ERROR! Direction must be 'x', 'y' or 'z'");
                return;
            }
            Some(index) => {
                if period == 0f32 {
                    self.set_param("PERIODIC_TRAN", index, 0);
                } else {
                    self.set_param(
                        "PERIODIC_TRAN",
                        index,
                        conver2u8(
                            period as f64,
                            Vec::from(self.device.period_limit.limit[0]),
                            Some(1),
                        ),
                    );
                }
                self.__send("PERIODIC_TRAN", Some(index as u8), None);
            }
        }
    }

    /// Make the robot move periodically
    /// 使机器狗周期性平动
    pub fn periodic_tran(&mut self, direction: &str, period: f32) {
        self.__periodic_tran(direction, period);
    }

    /// Make the robot move periodically (Multiple Directions)
    /// 使机器狗周期性平动 (多个方向)
    pub fn periodic_tran_all(&mut self, directions: Vec<&str>, periods: Vec<f32>) {
        if directions.len() != periods.len() {
            println!("ERROR! The length of direction and periods does not match!");
            return;
        }

        for i in 0..directions.len() {
            self.__periodic_tran(directions[i], periods[i]);
        }
    }

    /// Make the robot marks time
    /// 使机器狗原地踏步
    pub fn mark_time(&mut self, data: i32) {
        if let Some(value) = self.common_params.get_mut("MarkTime") {
            if data == 0 {
                value[1] = 0;
            } else {
                value[1] = conver2u8(
                    data as f64,
                    Vec::from(&mut self.device.mark_time_limit.limit),
                    Some(1),
                );
            }
        }

        self.__send("MarkTime", None, None);
    }

    /// Change the step frequency of the robot
    /// 改变机器狗的踏步频率
    pub fn pace(&mut self, mode: &str) {
        let mode_int: u8;

        match mode.to_lowercase().as_str() {
            "normal" => mode_int = 0x00,
            "slow" => mode_int = 0x01,
            "high" => mode_int = 0x02,
            _ => {
                println!("ERROR! Illegal Value!");
                return;
            }
        }

        self.set_param("MOVE_MODE", 1, mode_int);
        self.__send("MOVE_MODE", None, None);
    }

    /// Set the walking mode of the robot dog
    /// 设置机器狗的行走方式
    pub fn gait_type(&mut self, mode: &str) {
        let mode_int: u8;

        match mode.to_lowercase().as_str() {
            "trot" => mode_int = 0x00,
            "walk" => mode_int = 0x01,
            "high_walk" => mode_int = 0x02,
            "slow_trot" => mode_int = 0x03,
            _ => {
                println!("ERROR! Illegal Value!");
                return;
            }
        }

        self.set_param("GAIT_TYPE", 1, mode_int);
        self.__send("GAIT_TYPE", None, None);
    }

    /// Turn on / off the self stable state of the robot dog
    /// 开启/关闭机器狗自稳状态
    pub fn imu(&mut self, mode: u8) {
        if mode != 0 && mode != 1 {
            println!("ERROR! Illegal Value!");
            return;
        }

        self.set_param("IMU", 1, mode);
        self.__send("IMU", None, None);
    }

    /// Turn on / off the action status of the robot dog cycle
    /// 开启/关闭机器狗循环做动作状态
    pub fn perform(&mut self, mode: u8) {
        if mode != 0 && mode != 1 {
            println!("ERROR! Illegal Value!");
            return;
        }

        self.set_param("PERFORM", 1, mode);
        self.__send("PERFORM", None, None);
    }

    /// Adjust the steering gear rotation speed,
    /// only effective when control the steering gear separately
    /// 调节舵机转动速度，只在单独控制舵机的情况下有效
    pub fn motor_speed(&mut self, mut speed: u8) {
        if speed == 0 {
            speed = 1;
        }

        self.set_param("MOTOR_SPEED", 1, speed);
        self.__send("MOTOR_SPEED", None, None);
    }

    pub fn bt_rename(&mut self, _name: &str) {
        println!("bt_rename is not supported.")
    }

    /// Read the angle of the motor
    /// 读取15个舵机的角度
    pub fn read_motor(&mut self) -> Vec<f64> {
        self.__read(self.common_params["MOTOR_ANGLE"][0], Some(15));
        let mut angle = Vec::new();

        if self.__unpack(None) {
            for i in 0..self.rx_count {
                if i < 12 {
                    angle.push(round_to_2_decimal_places(conver2float(
                        self.rx_data[i],
                        self.device.motor_limit.limit[i % 3],
                    )));
                } else {
                    angle.push(round_to_2_decimal_places(conver2float(
                        self.rx_data[i],
                        self.device.motor_limit.limit[i - 9],
                    )));
                }
            }
        }

        angle
    }

    pub fn read_battery(&mut self) -> u8 {
        self.__read(self.common_params["BATTERY"][0], Some(1));

        let mut battery = 0u8;

        if self.__unpack(None) {
            battery = self.rx_data[0];
        }

        battery
    }

    pub fn read_firmware(&mut self) -> String {
        self.__read(self.common_params["FIRMWARE_VERSION"][0], Some(10));

        let mut firmware_version = String::from("Null");

        if self.__unpack(None) {
            let data = &self.rx_data[0..10];

            match std::str::from_utf8(data) {
                Ok(s) => firmware_version = s.trim_matches('\0').to_string(),
                Err(e) => println!("{}", e),
            }
        }

        firmware_version
    }

    pub fn read_roll(&mut self) -> f64 {
        self.__read(self.common_params["ROLL"][0], Some(4));

        let mut roll = 0.0;
        if self.__unpack(None) {
            roll = byte2float(&self.rx_data);
        }

        round_to_2_decimal_places(roll as f64)
    }

    pub fn read_pitch(&mut self) -> f64 {
        self.__read(self.common_params["PITCH"][0], Some(4));

        let mut pitch = 0.0;
        if self.__unpack(None) {
            pitch = byte2float(&self.rx_data);
        }

        round_to_2_decimal_places(pitch as f64)
    }

    pub fn read_yaw(&mut self) -> f64 {
        self.__read(self.common_params["YAW"][0], Some(4));

        let mut yaw = 0.0;
        if self.__unpack(None) {
            yaw = byte2float(&self.rx_data);
        }

        round_to_2_decimal_places(yaw as f64)
    }

    fn __unpack(&mut self, timeout: Option<f64>) -> bool {
        let timeout = timeout.unwrap_or(1.0);

        let start_time = std::time::Instant::now();
        let mut rx_msg = Vec::new();
        loop {
            let elapsed_time = start_time.elapsed().as_secs_f64();
            if elapsed_time > timeout {
                return false;
            }

            let mut data = vec![0; 1];

            match self.port.read_exact(&mut data) {
                Ok(()) => {
                    // Can read 1 byte data
                }
                Err(_) => {
                    // No data left
                    return false;
                }
            }

            for num in data {
                rx_msg.push(num);
                match self.rx_flag {
                    0 => {
                        if num == 0x55 {
                            self.rx_flag = 1;
                        } else {
                            self.rx_flag = 0;
                        }
                    }
                    1 => {
                        if num == 0x00 {
                            self.rx_flag = 2;
                        } else {
                            self.rx_flag = 0;
                        }
                    }
                    2 => {
                        self.rx_len = num as usize;
                        self.rx_flag = 3;
                    }
                    3 => {
                        self.rx_type = num;
                        self.rx_flag = 4;
                    }
                    4 => {
                        self.rx_addr = num;
                        self.rx_flag = 5;
                        self.rx_count = 0;
                    }
                    5 => {
                        if self.rx_count == ((self.rx_len - 9) as u8).into() {
                            self.rx_data[self.rx_count] = num;
                            self.rx_flag = 6;
                        } else if self.rx_count < ((self.rx_len - 9) as u8).into() {
                            self.rx_data[self.rx_count] = num;
                            self.rx_count += 1;
                        }
                    }
                    6 => {
                        let mut rx_check: u8 = 0;
                        for &i in &self.rx_data[0..(self.rx_len - 8)] {
                            rx_check = rx_check.wrapping_add(i);
                        }
                        let sum_data = (self.rx_len as u16
                            + self.rx_type as u16
                            + self.rx_addr as u16
                            + rx_check as u16)
                            % 256;
                        rx_check = (255 - sum_data) as u8;
                        if num == rx_check {
                            self.rx_flag = 7;
                        } else {
                            self.rx_flag = 0;
                            self.rx_count = 0;
                            self.rx_addr = 0;
                            self.rx_len = 0;
                        }
                    }
                    7 => {
                        if num == 0x00 {
                            self.rx_flag = 8;
                        } else {
                            self.rx_flag = 0;
                            self.rx_count = 0;
                            self.rx_addr = 0;
                            self.rx_len = 0;
                        }
                    }
                    8 => {
                        if num == 0xAA {
                            self.rx_flag = 0;
                            if self.verbose {
                                println!("rx_data: {:?}", self.rx_data);
                            }
                            return true;
                        } else {
                            self.rx_flag = 0;
                            self.rx_count = 0;
                            self.rx_addr = 0;
                            self.rx_len = 0;
                        }
                    }
                    _ => {
                        // Handle unexpected rx_flag values
                        self.rx_flag = 0;
                    }
                }
            }
        }
    }

    pub fn set_move_mintime(&mut self, mintime: f64) {
        self.mintime = mintime;
    }

    pub fn upgrade(&mut self, _filename: &str) {
        println!("upgrade is not supported.")
    }

    // TODO: Add more functions from xgolib.py line 758-954

    /// ############# RIDER ################

    pub fn rider_move_x(&mut self, speed: f64, runtime: f64) {
        self.set_param(
            "VX",
            1,
            conver2u8(speed, vec![self.device.vx_limit.limit], None),
        );
        self.__send("VX", None, None);

        if runtime > 0.0 {
            thread::sleep(Duration::from_secs_f64(runtime));
            self.set_param("VX", 1, 0);
            self.__send("VX", None, None);
        }
    }

    pub fn rider_turn(&mut self, speed: f64, runtime: f64) {
        self.set_param(
            "VYAW",
            1,
            conver2u8(speed as f64, vec![self.device.vyaw_limit.limit], None),
        );
        self.__send("VYAW", None, None);

        if runtime > 0.0 {
            thread::sleep(Duration::from_secs_f64(runtime));
            self.set_param("VYAW", 1, 0);
            self.__send("VYAW", None, None);
        }
    }

    pub fn rider_reset_odom(&mut self) {
        self.set_param("SET_ORIGIN", 1, 1);
        self.__send("SET_ORIGIN", None, None);
    }

    pub fn rider_action(&mut self, action_id: i32, wait: Option<bool>) {
        let wait = wait.unwrap_or(false);

        if action_id == 0 || action_id > 255 {
            println!("ERROR! Illegal Action ID!");
        }

        self.set_param("ACTION", 1, action_id as u8);
        self.__send("ACTION", None, None);

        if wait {
            if self.device.action_time.action_time.contains_key(&action_id) {
                thread::sleep(Duration::from_secs_f64(
                    self.device.action_time.action_time[&action_id].into(),
                ));
            }
        }
    }

    pub fn rider_balance_roll(&mut self, mode: u8) {
        if mode != 0 && mode != 1 {
            println!("ERROR! Illegal Value!");
            return;
        }

        self.set_param("IMU", 1, mode);
        self.__send("IMU", None, None);
    }

    pub fn rider_perform(&mut self, mode: u8) {
        if mode != 0 && mode != 1 {
            println!("ERROR! Illegal Value!");
            return;
        }

        self.set_param("PERFORM", 1, mode);
        self.__send("PERFORM", None, None);
    }

    pub fn rider_height(&mut self, data: f32) {
        self.__translation("z", data);
    }

    pub fn rider_roll(&mut self, data: f32) {
        self.__attitude("r", data);
    }

    pub fn rider_periodic_roll(&mut self, period: f32) {
        self.__periodic_rotate("r", period);
    }

    pub fn rider_periodic_z(&mut self, period: f32) {
        self.__periodic_tran("z", period);
    }

    pub fn rider_read_battery(&mut self) -> u8 {
        self.__read(self.common_params["BATTERY"][0], Some(1));
        let mut battery = 0;
        if self.__unpack(None) {
            battery = self.rx_data[0];
        }
        battery
    }

    pub fn rider_read_firmware(&mut self) -> &str {
        self.__read(self.common_params["FIRMWARE_VERSION"][0], Some(10));
        let mut firmware_version = "Null";
        if self.__unpack(None) {
            let bytes = &self.rx_data[0..10];
            firmware_version = std::str::from_utf8(&bytes).unwrap().trim();
        }
        firmware_version
    }

    pub fn rider_read_roll(&mut self) -> f64 {
        self.__read(self.common_params["ROLL"][0], Some(4));

        let mut roll = 0.0;
        if self.__unpack(None) {
            roll = byte2float(&self.rx_data);
        }

        round_to_2_decimal_places(roll as f64)
    }

    pub fn rider_read_pitch(&mut self) -> f64 {
        self.__read(self.common_params["PITCH"][0], Some(4));
        let mut pitch = 0.0;
        if self.__unpack(None) {
            pitch = byte2float(&self.rx_data);
        }
        round_to_2_decimal_places(pitch as f64)
    }

    pub fn rider_read_yaw(&mut self) -> f64 {
        self.__read(self.common_params["YAW"][0], Some(4));
        let mut yaw = 0.0;
        if self.__unpack(None) {
            yaw = byte2float(&self.rx_data);
        }
        round_to_2_decimal_places(yaw as f64)
    }

    pub fn rider_read_imu_float(&mut self, direction: &str) -> i16 {
        match direction {
            "roll" => self.__read(0x66, Some(2)),
            "pitch" => self.__read(0x67, Some(2)),
            "yaw" => self.__read(0x68, Some(2)),
            _ => {
                return 0;
            }
        }

        if self.__unpack(None) {
            byte2short(&self.rx_data);
        }

        0
    }

    pub fn rider_reset(&mut self) {
        self.reset()
    }

    pub fn rider_upgrade(&mut self, filename: &str) {
        self.upgrade(filename);
    }

    pub fn rider_led(&mut self, index: u8, color: u8) {
        self.set_param("LED_COLOR", 0, 0x68 + index);
        self.set_param("LED_COLOR", 1, color);
        self.set_param("LED_COLOR", 2, color);
        self.set_param("LED_COLOR", 3, color);
        self.__send("LED_COLOR", None, Some(3));
    }
}
