use crate::xgolib::XGO;

pub mod types;
pub mod xgolib;

pub fn test_move() {
    let mut xgo = XGO::new(String::from("/dev/ttyAMA0"), 115_200, false);
    println!("battery: {}", xgo.read_battery());

    xgo.rider_move_x(0.5, 0.1);
    xgo.rider_move_x(0f64, 0f64);
    xgo.rider_turn(0f64, 0f64);
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
