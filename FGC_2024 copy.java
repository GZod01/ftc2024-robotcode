package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.robot.Robot;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@TeleOp(name = "FGC2024", group = "WeRobot")
public class FGC_2024 extends LinearOpMode {


    private DcMotorEx rm;
    private DcMotorEx lm;
    private IMU imu;


    @Override
    public void runOpMode() throws InterruptedException {


        telemetry.addData("Status"," Initialized");
        telemetry.update();

        lm = hardwareMap.get(DcMotorEx.class, "lm");
        rm = hardwareMap.get(DcMotorEx.class, "rm");
        rm.setDirection(DcMotor.Direction.REVERSE);
        //lm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //rm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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
	    YawPitchRollAngle robotOrientation;
        double Yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);

        waitForStart();
        while(opModeIsActive()){
            robotOrientation = imu.getRobotYawPitchRollAngles();
            Yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);
            float x = gamepad1.left_stick_x; // abscisse joystick gauche
            double y = gamepad1.left_stick_y; // ordonnées joystick gauche
            double lpower = 0.0; //puissance moteur gauche
            double rpower = 0.0; //puissance moteur droit

            lpower = ((1+x)*Math.signum(y))/1.5;
            rpower = ((1-x)*Math.signum(y))/1.5;

            if ( Math.abs(x)==1){
                lpower = 0.75*Math.signum(x);
                rpower = -lpower;
            }
            
            if(gamepad1.right_trigger == 0){
                lpower /=3;
                rpower /=3;
            }else{
                lpower = lpower*gamepad1.right_trigger;
                rpower = rpower*gamepad1.right_trigger;
            }
            
            rm.setPower(rpower);
            lm.setPower(lpower);
            telemetry.addData("r", rpower);
            telemetry.addData("l", lpower);
            telemetry.addData("Yaw ", Yaw);
            telemetry.update();
        }
	}
}
