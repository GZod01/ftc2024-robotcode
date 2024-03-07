package org.firstinspires.ftc.teamcode;

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

@Autonomous

public class ftc2024_autonome_api extends LinearOpMode {
    private DcMotor rm;
    private DcMotor lm;
    private DcMotor harvestmotor;
    private IMU imu;
    private final double wheel_width = 9.0e-2; // metres
    private final double wheel_perimeter = Math.PI * wheel_width;
    private final double tour_par_minutes = 300.0;
    private final double ground_tiles_width = 61.0e-2; // metres
    /*
     * return a metre/sec speed
     * @param motor_speed = double between 0 and 1
     */
    public double getSpeedFromMotorSpeed(double motor_speed = 1.0){
      double speed_tour_par_minutes = this.tour_par_minutes*motor_speed;
      double speed = (speed_tour_par_minutes/60)*this.wheel_perimeter;
      return speed;
    }
    public double time_for_dist(double dist, double motor_speed=1.0){
      double speed = getSpeedFromMotorSpeed(motor_speed);
      return (dist/speed);
    }
    public void forward(double n_tiles, double motor_speed = 1.0){
      double total_time = time_for_dist(n_tiles*ground_tiles_width, motor_speed);
      double start_time = runtime.seconds();
      while( opModeIsActive() && ((runtime.seconds()-start_time)<total_time)){
        lm.setPower(motor_speed);
        rm.setPower(motor_speed);
      }
      lm.setPower(0);
      rm.setPower(0);
    }
    public void backward(double n_tiles, double motor_speed= 1.0){
        forward(n_tiles, -motor_speed);
    }
    public void harvest(double motor_speed=0.0){
        harvestmotor.setPower(motor_speed);
    }
    public void rotate(double angle, double motor_speed=1.0){
        double start_yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
        angle = 200.0;
        double anglerad = Math.toRadians(angle);
        angle = Math.toDegrees(Math.atan2(Math.sin(anglerad),Math.cos(anglerad)));
        double left_multiplier = -( (double) Math.signum(angle));
        double right_multiplier = ((double) Math.signum(angle));
        double m_power = motor_speed;
        while(opModeIsActive() && (Math.abs(robotOrientation.getYaw(AngleUnit.DEGREES) - start_yaw) < Math.abs(angle)){
            m_power = (Math.abs(robotOrientation.getYaw(AngleUnit.DEGREES)-start_yaw));//relative 
            lm.setPower(left_multiplier*m_power);
            rm.setPower(right_multiplier*m_power);
        }
        lm.setPower(0);
        rm.setPower(0);
    }

    @Override

    public void runOpMode() {
        lm = hardwareMap.get (DcMotor.class, "blm");
        rm = hardwareMap.get (DcMotor.class, "brm");
        harvestmotor = hardwareMap.get(DcMotor.class, "flm");

        rm.setDirection(DcMotor.Direction.REVERSE);
        
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
            new IMU.Parameters(
                new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
            )
            );
        imu.resetYaw();
        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();
        double Yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);
        double yaw_sortie = 0.0;

        waitForStart();

        if(opModeIsRunning()){
            forward(0.5);
            rotate(-90.0);
            forward(1.5);
            harvest(-1);
            backward(1);
            harvest(0);
    }
}
