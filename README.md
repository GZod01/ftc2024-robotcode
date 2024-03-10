# The 2024 [FIRST®](https://firstinspires.org) Tech Challenge robot competition WeRobot code

## Requirements
Work only with Rev Robotics Control Hub designed for [FIRST®](https://firstinspires.org) Tech Challenge competitions

## How To use?
Download the code and use it...

## How To Use the FTC2024WeRobotControl tool?
In your robot code import the class, then in the runOpMode function of your robot code create instance of the FTC2024WeRobotControl (e.g. `FTC2024WeRobotControl robot = new FTC2024WeRobotControl()`) then you can use this instance to move the robot.

E.G.
```java
double speed_for_motors = 1.0;
double number_or_tiles = 3.0;
double angle_to_rotate = 50;  // in degrees
FTC2024WeRobotControl robot = new FTC2024WeRobotControl()
robot.forward(number_or_tiles, speed_for_motors);
robot.rotate(angle_to_rotate, speed_for_motors);
```

Read the FTC2024WeRobotControl.java file to get complete docs.
