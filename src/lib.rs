use std::thread;
use std::time::Duration;
use crate::xgolib::XGO;

pub mod types;
pub mod xgolib;

pub fn test_move() {
    let mut xgo = XGO::new(String::from("/dev/ttyAMA0"), 115_200, true);
    println!("battery: {}", xgo.rider_read_battery());

    xgo.rider_move_x(1.0, 1.0);
    xgo.rider_periodic_roll(10f32);
    xgo.rider_height(115f32);

    thread::sleep(Duration::from_secs(2));
    xgo.reset();
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let result = test_move();
    }
}
