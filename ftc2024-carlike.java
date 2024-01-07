package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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


@TeleOp(name="WeRobot: FTC2024 Carlike", group="WeRobot")
public class Werobot_FTC2024_carlike extends LinearOpMode {
	private DcMotor rm;
	private DcMotor lm;
	private IMU imu;
	private double helloexp(double t){
		return (Math.exp(5*t)-1)/(Math.exp(5)-1);
	}
	@Override
	public void runOpMode() throws InterruptedException {
		float x;
		double y;
		double t;
		double t2;
		double t3;
		String mode = "normal";
		boolean already_a = false;
		telemetry.addData("Status", "Initialized");
		telemetry.update();
		lm = hardwareMap.get(DcMotor.class, "blm");
		rm = hardwareMap.get(DcMotor.class, "brm");


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
		rm.setDirection(DcMotorSimple.Direction.REVERSE);
		//telemetry.addData("Mode", "calibrating...");
		//telemetry.update();

		// make sure the imu gyro is calibrated before continuing.
		//while (!isStopRequested() && !imu.isGyroCalibrated())
		//{
		//	sleep(50);
		//	idle();
		//}

		telemetry.addData("Mode", "waiting for start");
		//telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
		telemetry.update();
		waitForStart();


		while (opModeIsActive()) {
			x = gamepad1.left_stick_x;
			y = gamepad1.left_stick_y;
			t= gamepad1.right_trigger;
			t2 = helloexp(t);
			t3 = helloexp(Math.sqrt(Math.pow(x,2)+Math.pow(y,2)));
			telemetry.addData("Status", "Running");
			if(gamepad1.a && !already_a){
				if(mode=="normal"){
					mode="tank";
				}else if(mode=="tank"){
					mode = "essaifranck";
				}else if (mode == "essaifranck"){
					mode = "elina";
				}else{
					mode="normal";
				}
				already_a = true;
			}
			if(!gamepad1.a && already_a){
				already_a = false;
			}
			double lpower = 0.0;
			double rpower = 0.0;
			if(mode=="normal"){
				double ysign = y>0?1.0:(y<0?-1.0:0.0);
				double xsign = x>0?1.0:(x<0?-1.0:0.0);
				lpower = -ysign * t + (xsign-2*x)*t;
				rpower = ysign * t + (xsign-2*x)*t;
			}
			else if (mode=="tank"){
				lpower = -y;
				rpower = gamepad1.right_stick_y;
			}
			else if (mode=="essaifranck"){
				double a = (-y+x)/Math.pow(2,1/2);
				double b = (-y-x)/Math.pow(2,1/2);
				double vmean = (Math.abs(a)+Math.abs(b))/2;
				lpower = (a/vmean)*t2;
				rpower = (b/vmean)*t2;
			}
			else if (mode=="elina"){
				double a = (-y+x)/Math.pow(2,1/2);
				double b = (-y-x)/Math.pow(2,1/2);
				double vmean = (Math.abs(a)+Math.abs(b))/2;
				lpower = (a/vmean)*t3;
				rpower = (b/vmean)*t3;
			}
			if(!(gamepad1.left_bumper)){
				lpower/=3;
				rpower/=3;
			}
			lm.setPower(lpower);
			rm.setPower(rpower);

			telemetry.addData("x",x);
			telemetry.addData("y",y);
			telemetry.addData("lpow",lpower);
			telemetry.addData("rpow",rpower);
			telemetry.addData("ltrigg",t);
			telemetry.addData("t2",t2);
			telemetry.addData("mode",mode);
			// Create an object to receive the IMU angles
			YawPitchRollAngles robotOrientation;
			robotOrientation = imu.getRobotYawPitchRollAngles();

			// Now use these simple methods to extract each angle
			// (Java type double) from the object you just created:
			double Yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);
			double Pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
			double Roll  = robotOrientation.getRoll(AngleUnit.DEGREES);
			telemetry.addData("yaw",Yaw);

			telemetry.update();
		}
	}
}
