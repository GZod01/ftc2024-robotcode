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

public class ftc2024_autonome extends LinearOpMode {
	private DcMotor rm;
	private DcMotor lm;
	private ElapsedTime     runtime = new ElapsedTime();

	@Override
	public void runOpMode() {
		lm = hardwareMap.get(DcMotor.class, "blm");
		rm = hardwareMap.get(DcMotor.class, "brm");
		telemetry.addData("Status", "Initialized");
		telemetry.update();
        boolean mode = true;
		// Wait for the game to start (driver presses PLAY)
		waitForStart();

        runtime.reset();
        if (mode){
            //mode Elina
            while (opModeIsActive() && (runtime.seconds() <= 3.0)) {
                leftMotor.setPower(1);
                rightMotor.setPower(-1);
                telemetry.addData("Leg 1", runtime.seconds());
                telemetry.update();
            }
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() <= 10.0)) {
                leftMotor.setPower(1);
                rightMotor.setPower(1);
                telemetry.addData("Leg 2", runtime.seconds());
                telemetry.update();
            }
        }
        else {
            //mode Aurelien
            while (opModeIsActive() && (runtime.seconds() <= 3.0)) {
                leftMotor.setPower(-1);
                rightMotor.setPower(-1);
                telemetry.addData("Leg 2", runtime.seconds());
                telemetry.update();
            }
        }
		// run until the end of the match (driver presses STOP

	}

}

