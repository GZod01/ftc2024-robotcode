package fr.werobot.ftc2024.robotcontrol;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class FTC2024WeRobotControl {
    /*
     * The Parent class {@see FTC2024WeRobotControl constructor}
     */
    private LinearOpMode Parent;
    /*
     * the wheel width in metres
     */
    private final double wheel_width = 9.0e-2; // metres
    /*
     * the wheel perimeter in meter
     */
    private final double wheel_perimeter = Math.PI * wheel_width;
    /*
     * the rpm at max power of the motors
     */
    private final double tour_par_minutes = 300.0;
    /*
     * the width size of the tiles on the ground in metres
     */
    private final double ground_tiles_width = 61.0e-2; // metres
    /*
     * construct the FTC2024WeRobotControl class, the WeRobot Robot Controller Class for the FTC2024
     * @param Parent = the parent class, use the "this" keyword if you are constructing the class directly in 
     */
    public FTC2024WeRobotControl(YawPitchhRollAngle Parent){
	this.Parent = Parent;
    }
    /*
     * return a metre/sec speed
     * @param motor_speed = (optional) double between 0 and 1; motor power; default to 1
     */
    public double getSpeedFromMotorSpeed(double motor_speed = 1.0){
	double speed_tour_par_minutes = this.tour_par_minutes*motor_speed;
	double speed = (speed_tour_par_minutes/60)*this.wheel_perimeter;
	return speed;
    }
    /*
     * return the needed time for a distance
     * @param dist = distance in metre
     * @param motor_speed = (optional) double between 0 and 1; motor power; default to 1
     */
    public double time_for_dist(double dist, double motor_speed=1.0){
	double speed = getSpeedFromMotorSpeed(motor_speed);
	return (dist/speed);
    }
    /*
     * go forward
     * @param n_tiles = the number of tiles (a double because it can be 0.5 or 1.5 etc.)
     * @param motor_speed = (optional) double between 0 and 1; motor power; default to 1
     */
    public void forward(double n_tiles, double motor_speed = 1.0){
	double total_time = time_for_dist(n_tiles*ground_tiles_width, motor_speed);
	double start_time = Parent.runtime.seconds();
	while( Parent.opModeIsActive() && ((Parent.runtime.seconds()-start_time)<total_time)){
	    Parent.lm.setPower(motor_speed);
	    Parent.rm.setPower(motor_speed);
	}
	Parent.lm.setPower(0);
	Parent.rm.setPower(0);
    }
    /*
     * go backward
     * @param n_tiles = the number of tiles (a double because it can be 0.5 or 1.5 etc.)
     * @param motor_speed = (optional) double between 0 and 1; motor power; default to 1
     */
    public void backward(double n_tiles, double motor_speed= 1.0){
	forward(n_tiles, -motor_speed);
    }
    /*
     * harvest
     * @param motor_speed = (optional) double between 0 and 1; motor power; default to 1
     */
    public void harvest(double motor_speed=0.0){
	Parent.harvestmotor.setPower(motor_speed);
    }
    /*
     * harvest
     * @param angle = the angle to rotate (in degrees)
     * @param motor_speed = (optional) double between 0 and 1; motor power; default to 1
     */
    public void rotate(double angle, double motor_speed=1.0){
        Parent.robotOrientation = Parent.imu.getRobotYawPitchRollAngles();
	double start_yaw = Parent.robotOrientation.getYaw(AngleUnit.DEGREES);
	angle = 200.0;
	double anglerad = Math.toRadians(angle);
	angle = Math.toDegrees(Math.atan2(Math.sin(anglerad),Math.cos(anglerad)));
	double left_multiplier = -( (double) Math.signum(angle));
	double right_multiplier = ((double) Math.signum(angle));
	double m_power = motor_speed;
	while(Parent.opModeIsActive() && (Math.abs(Parent.robotOrientation.getYaw(AngleUnit.DEGREES) - start_yaw) < Math.abs(angle)){
	    Parent.robotOrientation = Parent.imu.getRobotYawPitchRollAngles();
	    m_power = (Math.abs(Parent.robotOrientation.getYaw(AngleUnit.DEGREES)-start_yaw));//relative 
	    Parent.lm.setPower(left_multiplier*m_power);
	    Parent.rm.setPower(right_multiplier*m_power);
	}
	Parent.lm.setPower(0);
	Parent.rm.setPower(0);
    }
}
