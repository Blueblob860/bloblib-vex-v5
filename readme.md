# Bloblib

Hello there! Bloblib is an open source motion control library for the Vex V5 platform, written in Rust with Vexide. Bloblib aims to be a Rust library similar in structure and usage to the PROS library [LemLib](https://github.com/LemLib/LemLib/), but with extra features and some QOL changes.

> [!Warning]
> Bloblib was created and is currently maintained by team 934Z for our competition code this VRC season, as such there is no stable API to use yet.  
> Proceed with caution before committing to using Bloblib please, and expect to have to make major changes if you update, thanks :D

## Features
Note that this feature list is currently unfinished and also incomplete please

[x] Pid Control
[x] Odometry (Tracking wheels optional)
[x] Turn/Swing to Face Heading
[x] Turn/Swing to Face Point
[x] Boomerang Move to Pose
[ ] Path Following via Stanley
[x] All Driver Control Functions from Lemlib

## Getting Started
First off, if you don't already have [Rust](rust-lang.org) and [vexide](vexide.dev) setup, start off with the [vexide getting started guide](vexide.dev/docs)

After getting Rust and vexide setup (I will assume that you read the vexide docs up to section 3), lets add Bloblib to the project. The following command will add the latest version of Bloblib to your project:
```bash
cargo add --git "https://github.com/Blueblob860/bloblib-vex-v5.git"
```
> [!Important]
> Since this dependency is based on the master branch, its a good idea to pin it to a certain commit in case someone else builds it on their device or you need to refresh dependencies  
> You can do this by appending `--rev <commit-hash>` to the end of the command, replacing `<commit-hash>` with the commit you are using

Now that Bloblib has been added to our project, we can set up a chassis. Lets start off with the drivetrain. First, off create the motors as a vector of our left and right side motors:
```rust
let left_motors: Vec<Motor> = vec![
    Motor::new(peripherals.port_1, Gearset::Blue, Direction::Forward)
    Motor::new(peripherals.port_2, Gearset::Blue, Direction::Forward)
    Motor::new(peripherals.port_3, Gearset::Blue, Direction::Forward)
];
let right_motors: Vec<Motor> = vec![
    Motor::new(peripherals.port_4, Gearset::Blue, Direction::Reverse)
    Motor::new(peripherals.port_5, Gearset::Blue, Direction::Reverse)
    Motor::new(peripherals.port_6, Gearset::Blue, Direction::Reverse)
]
```
Now we can create our Drivetrain like so:
```rust
let drivetrain: Drivetrain = Drivetrain::new(
    left_motors, right_motors, // Our left and right drive motors
    10.6, // Track Width, or the distance between the middle of our left and right wheels
    2.75, // The diameter of our wheels
    36.0/45.0, // Gear ratio from the motors to the wheels
    2.0 // Horizontal drift
);
```

Now we need to define the sensors we are using for odometry, this step is pretty similar to LemLib's apart from the creation of the sensors variable:

```rust
let sensors: Sensors = Sensors {
    imu: InertialSensor::new(peripherals.port_7),
    ..Default::default()
};
```

> [!Tip]
> Enabling the `default-field-values` feature allows you to omit the `Default::default()` at the end and makes the code look a little nicer

Now we can create the chassis (PIDs come after this don't worry):

```rust
let mut chassis: Chassis = Chassis::new(
    drivetrain,
    sensors,
    controller // Bloblib uses the controller for showing IMU calibration metrics
);
```

Now we can create our PIDs using `PidBuilder`, once again using `..Default::default()` to make our lives a little easier:

```rust
chassis.linear = PidBuilder {
    kp: 4.0, ki: 0.0, kd: 20.0
    ..Default::default()
}.into();
chassis.angular = PidBuilder {
    kp: 4.0, ki: 0.0, kd: 20.0
    ..Default::default()
}.into();
```

Now that we have our chassis we can finally run the competition loop (make sure to put the chassis in your robot struct!) To start the odom loop and calibrate our imu we can do the following:
```rust
let _ = chassis.calibrate(true).await;
```
The return value from the function is the task handle from the odom loop so that it doesn't prematurely drop or if you need to use it. Final thing for setup, don't forget to start the competition loop:
```rust
robot.compete().await;
```
From here, everything is pretty similar to base LemLib. One thing to note is that since our chassis has our controller variable, we can access it by doing
```rust
let controller = chassis.controller.write().await;
```
Or you could clone the `RwLock` into your robot struct, up to you.