# xgolib-rs

It is a rust port of [XGO-PythonLib](https://github.com/Xgorobot/XGO-PythonLib/).

Tested on XGO Rider

## Usage

```rust
let mut xgo = XGO::new(String::from("/dev/ttyAMA0"), 115_200, false);
println!("battery: {}", xgo.read_battery());

xgo.rider_move_x(0.5, 0.1);
xgo.rider_move_x(0f64, 0f64);
xgo.rider_turn(0f64, 0f64);
xgo.reset();
```

## License

MIT