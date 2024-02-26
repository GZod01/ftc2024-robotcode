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
        double yaw_sortie;
        double a;
        double a1;
        double b;
        double b1;
        double k;
        double k1;

        waitForStart();

        while (opModeIsActive()){
            double [] lm_p = {0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1};
            double [] rm_p = {-0.1,-0.2,-0.3,-0.4,-0.5,-0.6,-0.7,-0.8,-0.9,-1};
            for(int i = 0; i< lm_p.length; i++){
                while (opModeIsActive() && Yaw < 90){
                    lm.setPower = lm_p[i];
                    rm.setPower = rm_p[i];
                    Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
                    telemetry.addData("Yaw : ", Yaw);
                    telemetry.update();
                    yaw_sortie = Yaw;
                }
                telemetry.addData("Yaw sortie", yaw_sortie);
                telemetry.update();
                Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
                double [] x = Yaw - yaw_sortie;
                              
                
                
                /*if (i = 0) {
                    Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
                    a = yaw_sortie;
                    a1 = Yaw;
                }
                if (i = 1){
                    Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
                    b = yaw_sortie;
                    b1 = Yaw;
                }
                if (i = 2){
                    Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
                    double c = yaw_sortie;
                    double c1 = Yaw;
                }
                if (i = 3){
                    Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
                    double d = yaw_sortie;
                    double d1 = Yaw;
                }
                if (i = 4){
                    Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
                    double e = yaw_sortie;
                    double e1 = Yaw;
                }
                if (i = 5){
                    Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
                    double f = yaw_sortie;
                    double f1 = Yaw;
                }
                if (i = 6){
                    Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
                    double g = yaw_sortie;
                    double g1 = Yaw;
                }
                if (i = 7){
                    Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
                    double h = yaw_sortie;
                    double h1 = Yaw;
                }
                if (i = 8){
                    Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
                    double j = yaw_sortie;
                    double j1 = Yaw;
                }
                if (i = 9){
                    Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
                    k = yaw_sortie;
                    k1 = Yaw;
                }
                */
                imu.resetYaw();
            }
            

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