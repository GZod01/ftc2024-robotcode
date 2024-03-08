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

public class ftc2024_autonome_test extends LinearOpMode {
    private DcMotor rm;
    private DcMotor lm;
    private IMU imu;

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
