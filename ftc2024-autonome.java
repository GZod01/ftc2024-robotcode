package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous

public class HelloWorld_ElapsedTime extends LinearOpMode {
	private DcMotor leftMotor;
	private DcMotor rightMotor;
	private DcMotor arm;
	private Servo claw;
	private DigitalChannel touch;
	private Gyroscope imu; 
	private ElapsedTime     runtime = new ElapsedTime();

	@Override
	public void runOpMode() {
		imu = hardwareMap.get(Gyroscope.class, "imu");
		leftMotor = hardwareMap.get(DcMotor.class, "leftmotor");
		rightMotor = hardwareMap.get(DcMotor.class, "rightmotor");
		arm = hardwareMap.get(DcMotor.class, "arm");
		claw = hardwareMap.get(Servo.class, "claw");
		touch = hardwareMap.get(DigitalChannel.class, "touch");
		leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
		rightMotor.setDirection(DcMotor.Direction.REVERSE);

		telemetry.addData("Status", "Initialized");
		telemetry.update();
		// Wait for the game to start (driver presses PLAY)
		waitForStart();

		// run until the end of the match (driver presses STOP)

		runtime.reset();
		while (opModeIsActive() && (runtime.seconds() <= 3.0)) {
			leftMotor.setPower(1);
			rightMotor.setPower(1);
			telemetry.addData("Leg 1", runtime.seconds());
			telemetry.update();
		}

		runtime.reset();
		while (opModeIsActive() && (runtime.seconds() <= 3.0)) {
			leftMotor.setPower(-1);
			rightMotor.setPower(-1);
			telemetry.addData("Leg 2", runtime.seconds());
			telemetry.update();
		}

	}

}
