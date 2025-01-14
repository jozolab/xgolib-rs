use std::collections::HashMap;

macro_rules! pub_struct {
    ($name:ident {$($field:ident: $t:ty,)*}) => {
        #[derive(Debug, Clone, PartialEq)] // ewww
        pub struct $name {
            $(pub $field: $t),*
        }
    }
}

pub_struct!(TranslationLimit {
    x_limit: f64,
    y_limit: f64,
    z_limit: [f64; 2],
});

pub_struct!(AttitudeLimit {
    roll_limit: f64,
    pitch_limit: f64,
    yaw_limit: f64,
});

pub_struct!(LegLimit {
    x_limit: f64,
    y_limit: f64,
    z_limit: [f64; 2],
});

pub_struct!(MotorLimit {
    limit: [[f64; 2]; 6],
});

pub_struct!(PeriodLimit {
    limit: [[f64; 2]; 1],
});

pub_struct!(MarkTimeLimit { limit: [f64; 2], });

pub_struct!(VXLimit { limit: f64, });

pub_struct!(VYLimit { limit: f64, });

pub_struct!(VYAWLimit { limit: f64, });

pub_struct!(ARMLimit {
    limit: [[f64; 2]; 4],
});

pub_struct!(ActionTime {
    action_time: HashMap<i32, i32>,
});

pub_struct!(XGODevice {
    translation_limit: TranslationLimit,
    attitude_limit: AttitudeLimit,
    leg_limit: LegLimit,
    motor_limit: MotorLimit,
    period_limit: PeriodLimit,
    mark_time_limit: MarkTimeLimit,
    vx_limit: VXLimit,
    vy_limit: VYLimit,
    vyaw_limit: VYAWLimit,
    arm_limit: ARMLimit,
    action_time: ActionTime,
});
