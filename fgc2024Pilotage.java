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

@TeleOp(name = "WeRobot: FTC2024 NEW! Carlike", group = "WeRobot")
public class WEROBOT_FTC2024_New_carlike extends LinearOpMode {
    
    private DcMotorEx rm;
    private DcMotorEx lm;


    @Override
    public void runOpMode() throws InterruptedException {
        
        float x = gamepad1.left_stick_x; // abscisse joystick gauche
        double y = gamepad1.left_stick_y; // ordonnées joystick gauche
        double lpower = 0.0; //puissance moteur gauche
        double rpower = 0.0; //puissance moteur droit

        telemetry.addData("Status"," Initialized");

        lm = hardwareMap.get(DcMotorEx.class, "lm");
	    rm = hardwareMap.get(DcMotorEx.class, "rm");
        lm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	    rm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	    lm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
	    rm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        lpower = ((1+x)*Math.signum(y))/2;
        rpower = ((1-x)*Math.signum(y))/2;

        if ( x=1 || x=-1 ){
            lpower = 1*Math.signum(x);
            rpower = - lpower;
        }

        lpower = lpower*gamepad1.left_trigger;
        rpower = lpower*gamepad1.left_trigger;

        rm.setPower(rpower);
        lm.setPower(lpower);
    }
}