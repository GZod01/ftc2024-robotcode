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
    private IMU imu;
    private static double wheel_width = 9.0e-2; // metres
    private static double wheel_perimeter = Math.PI * wheel_width;
    private static double tour_par_minutes = 300.0;
    private static double ground_tiles_width = 61.0e-2; // metres
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
      double total_time = time_for_dist(n_tiles, motor_speed);
      double start_time = runtime.seconds();
      while( opModeIsActive() && ((runtime.seconds()-start_time)<total_time)){
        lm.setPower(motor_speed);
        rm.setPower(motor_speed);
      }
      lm.setPower(0);
      rm.setPower(0);
    }

    @Override

    public void runOpMode() {
        lm = hardwareMap.get (DcMotor.class, "blm");
        rm = hardwareMap.get (DcMotor.class, "brm");

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

        while (opModeIsActive()){
            double [] lm_p = {0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1};
            double [] rm_p = {-0.1,-0.2,-0.3,-0.4,-0.5,-0.6,-0.7,-0.8,-0.9,-1};
            double [] x = new double[lm_p.length];
            for(int i = 0; i< 9; i++){
                while (opModeIsActive() && Yaw < 90){
                    lm.setPower(lm_p[i]);
                    rm.setPower(rm_p[i]);
                    robotOrientation = imu.getRobotYawPitchRollAngles();
                    Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
                    telemetry.addData("Yaw ", Yaw);
                    telemetry.update();
                    yaw_sortie = Yaw;
                }
                telemetry.addData("Yaw sortie", yaw_sortie);
                telemetry.update();
                Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
                x [i]= Yaw - yaw_sortie;
                
                imu.resetYaw();
                Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
                telemetry.addData("Yaw", Yaw);
                telemetry.update();
            }
            while (opModeIsActive()){
                telemetry.addData("0.1", x[0]);
                telemetry.addData("0.2", x[1]);
                telemetry.addData("0.3", x[2]);
                telemetry.addData("0.4", x[3]);
                telemetry.addData("0.5", x[4]);
                telemetry.addData("0.6", x[5]);
                telemetry.addData("0.7", x[6]);
                telemetry.addData("0.8", x[7]);
                telemetry.addData("0.9", x[8]);
                telemetry.addData("1", x[9]);
            }
        }
    }
}
