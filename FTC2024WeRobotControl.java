package org.firstinspires.ftc.teamcode;//a tester car pas sur que ça fonctionne

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
    private Ftc2024_autonome_api Parent;
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
    
    private ElapsedTime timer;
     
    private YawPitchRollAngles robotOrientation;
    
    /*
     * construct the FTC2024WeRobotControl class, the WeRobot Robot Controller Class
     * for the FTC2024
     * 
     * @param Parent = the parent class, use the "this" keyword if you are
     * constructing the class directly in
     */

    public FTC2024WeRobotControl(Ftc2024_autonome_api Parent) {
        this.Parent = Parent;
        this.timer = new ElapsedTime();
    }

    /*
     * return a metre/sec speed
     * 
     * @param motor_speed = (optional) double between 0 and 1; motor power; default
     * to 1
     */
    public double getSpeedFromMotorSpeed(double motor_speed) {
        testMotorSpeed(motor_speed);
        double speed_tour_par_minutes = this.tour_par_minutes * motor_speed;
        double speed = (speed_tour_par_minutes / 60) * this.wheel_perimeter;
        return speed;
    }

    /*
     * test if motorSpeed is >= 0
     */
    public void testMotorSpeed(double motor_speed){
        if(!((double)motor_speed>=0.0)){
            throw new Exception("Motor Speed MUST be >= 0");
        }
    }
    /*
     * return the needed time for a distance
     * 
     * @param dist = distance in metre
     * 
     * @param motor_speed = (optional) double between 0 and 1; motor power; default
     * to 1
     */
    public double time_for_dist(double dist, double motor_speed) {
        testMotorSpeed(motor_speed);
        double speed = getSpeedFromMotorSpeed(motor_speed);
        return (dist / speed);
    }

    /*
     * go forward
     * 
     * @param n_tiles = the number of tiles (a double because it can be 0.5 or 1.5
     * etc.)
     * 
     * @param motor_speed = (optional) double between 0 and 1; motor power; default
     * to 1
     */
    public void forward(double n_tiles, double motor_speed) {
        testMotorSpeed(motor_speed);
        double total_time = time_for_dist(n_tiles * ground_tiles_width, motor_speed);
        timer.reset();
        while (Parent.opModeIsActive() && timer.seconds() < total_time) {
            Parent.lm.setPower(motor_speed);
            Parent.rm.setPower(motor_speed);
        }
        Parent.lm.setPower(0);
        Parent.rm.setPower(0);
    }

    public void forward(double n_tiles){
        this.forward(n_tiles,1);
    }

    /*
     * go backward
     * 
     * @param n_tiles = the number of tiles (a double because it can be 0.5 or 1.5
     * etc.)
     * 
     * @param motor_speed = (optional) double between 0 and 1; motor power; default
     * to 1
     */
    public void backward(double n_tiles, double motor_speed) {
        testMotorSpeed(motor_speed);
        forward(n_tiles, -motor_speed);
    }
    
    public void backward(double n_tiles){
        this.backward(n_tiles,1);
    }

    /*
     * harvest
     * 
     * @param motor_speed = (optional) double between 0 and 1; motor power; default
     * to 1
     */
    public void harvest(double motor_speed) {
        testMotorSpeed(motor_speed);
        Parent.harvestmotor.setPower(motor_speed);
    }
    public void harvest(){
        this.harvest(1);
    }

    /*
     * harvest
     * 
     * @param angle = the angle to rotate (in degrees)
     * 
     * @param motor_speed = (optional) double between 0 and 1; motor power; default
     * to 1
     */
    public void rotate(double angle, double motor_speed){
        testMotorSpeed(motor_speed);
        robotOrientation = Parent.imu.getRobotYawPitchRollAngles();
        double start_yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
        angle = 200.0;
        double anglerad = Math.toRadians(angle);
        angle = Math.toDegrees(Math.atan2(Math.sin(anglerad),Math.cos(anglerad)));
        double left_multiplier = -((double) Math.signum(angle));
        double right_multiplier = ((double) Math.signum(angle));
        double m_power = motor_speed;
        while(Parent.opModeIsActive() && (Math.abs(robotOrientation.getYaw(AngleUnit.DEGREES) - start_yaw) < Math.abs(angle))){
            robotOrientation = Parent.imu.getRobotYawPitchRollAngles();
            m_power = (Math.abs(robotOrientation.getYaw(AngleUnit.DEGREES)-start_yaw));//relative 
            Parent.lm.setPower(left_multiplier*m_power);
            Parent.rm.setPower(right_multiplier*m_power);
        }
        Parent.lm.setPower(0);
        Parent.rm.setPower(0);
    }
    public void rotate(double angle){
        this.rotate(angle,1.0);
    }
}
