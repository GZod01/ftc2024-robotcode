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


    @Override
    public void runOpMode() throws InterruptedException {


	telemetry.addData("Status"," Initialized");
	telemetry.update();

	lm = hardwareMap.get(DcMotorEx.class, "lm");
	lm.setDirection(DcMotor.Direction.REVERSE);
	rm = hardwareMap.get(DcMotorEx.class, "rm");
	//lm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
	//rm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

	waitForStart();
	while(opModeIsActive()){
	    float x = gamepad1.left_stick_x; // abscisse joystick gauche
	    double y = gamepad1.left_stick_y; // ordonnées joystick gauche
	    double lpower = 0.0; //puissance moteur gauche
	    double rpower = 0.0; //puissance moteur droit

	    lpower = ((1+x)*Math.signum(y))/2;
	    rpower = ((1-x)*Math.signum(y))/2;

	    if (Math.abs(x)>0.8){
		lpower = 1*Math.signum(x);
		rpower = -lpower;
	    }

	    lpower = lpower*gamepad1.left_trigger;
	    rpower = lpower*gamepad1.left_trigger;

	    rm.setPower(rpower);
	    lm.setPower(lpower);
	    telemetry.update();
	}
    }
}
