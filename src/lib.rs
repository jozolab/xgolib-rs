use std::thread;
use std::time::Duration;
use crate::xgolib::XGO;

pub mod types;
pub mod xgolib;

pub fn test_functions() {
    let mut xgo = XGO::new("/dev/ttyAMA0", 115_200, true);
    println!("battery: {}", xgo.rider_read_battery());

    // Set LED
    println!("led: 1 -> [0, 0, 255], 2 -> [255, 255, 0]");
    xgo.rider_led(1, [0, 0, 255]);
    xgo.rider_led(2, [255, 255, 0]);
    thread::sleep(Duration::from_secs(2));
    xgo.rider_led(1, [0, 0, 0]);
    xgo.rider_led(2, [0, 0, 0]);
    xgo.rider_reset();

    // Set Balance
    println!("balance_roll: 1");
    xgo.rider_balance_roll(1);
    thread::sleep(Duration::from_secs(2));
    xgo.rider_reset();

    // Turn
    println!("turn: 90");
    xgo.rider_turn(90.0, 1.0);
    thread::sleep(Duration::from_secs(2));
    xgo.rider_reset();

    // Move
    println!("move_x: 0.2");
    xgo.rider_move_x(0.2, 1.0);
    thread::sleep(Duration::from_secs(2));
    xgo.rider_reset();

    // Periodic Roll
    println!("periodic_roll: 1");
    xgo.rider_periodic_roll(1.0);
    thread::sleep(Duration::from_secs(2));
    xgo.rider_reset();

    // Set Roll
    println!("roll: 8 / -8");
    xgo.rider_roll(8.0);
    thread::sleep(Duration::from_millis(1000));
    xgo.rider_roll(-8.0);
    thread::sleep(Duration::from_millis(1000));
    xgo.rider_reset();

    // Periodic Z
    println!("periodic_z: 1");
    xgo.rider_periodic_z(1.0);
    thread::sleep(Duration::from_secs(2));
    xgo.rider_reset();

    // Set Height
    println!("height: 115 -> 75");
    xgo.rider_height(115.0);
    thread::sleep(Duration::from_millis(1000));
    xgo.rider_height(75.0);
    thread::sleep(Duration::from_millis(1000));
    xgo.rider_reset();

    // Perform
    println!("perform: 1");
    xgo.rider_perform(1);
    thread::sleep(Duration::from_secs(2));
    xgo.rider_reset();

    // Action
    println!("action: 4");
    xgo.rider_action(4, None);
    thread::sleep(Duration::from_secs(2));
    xgo.rider_reset();
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        test_functions();
    }
}
