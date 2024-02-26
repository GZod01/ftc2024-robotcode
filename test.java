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
        lm = hardwareMap.get (DcMotor.class, "lm");
        rm = hardwareMap.get (DcMotor.class, "rm");

        rm.setDirection(DcMotorSimple.Direction.REVERSE);
        
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
        double Yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);
        double yaw_sortie1;
        double yaw_sortie2;
        double yaw_sortie3;
        double yaw_sortie4;

        waitForStart();

        while (opModeIsActive()){
            double [] lm_p = {0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1};
            double [] rm_p = {-0.1,-0.2,-0.3,-0.4,-0.5,-0.6,-0.7,-0.8,-0.9,-1};
            for(int i = 0; i< p_t_g.length; i++){
                while (opModeIsActive() && Yaw < 90){
                    lm.setPower = lm_p[i];
                    rm.setPower = rm_p[i];
                    Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
                    telemetry.addData("Yaw : ", Yaw);
                    telemetry.update();
                }
            }
        }
    }
}