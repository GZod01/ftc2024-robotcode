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
	

	public double time_for_dist(double speed, double dist){
		return (double) (dist/speed);
	}

	@Override
	public void runOpMode() {
		lm = hardwareMap.get(DcMotor.class, "blm");
		rm = hardwareMap.get(DcMotor.class, "brm");
		rm.setDirection(DcMotorSimple.Direction.REVERSE);
		telemetry.addData("Status", "Initialized");
		telemetry.update();
		double tour_par_minute = 300.0;
		double wheel_width = 9.0e-2;
		double wheel_rayon = (wheel_width)/2;
		double wheel_perimeter = wheel_rayon*2*Math.PI;
		double speed = (tour_par_minute/60)*wheel_perimeter;//dist per second
		boolean mode = false;
		// Wait for the game to start (driver presses PLAY)
		waitForStart();

		runtime.reset();
		if (mode){
			//mode Elina
			while (opModeIsActive() && (runtime.seconds() <= 41*maths.PI/4/speed)) {
				lm.setPower(1);
				rm.setPower(-1);
				telemetry.addData("Leg 1", runtime.seconds());
				telemetry.update();
			}
			runtime.reset();
			while (opModeIsActive() && (runtime.seconds() <= 121.92/speed)) {
				lm.setPower(1);
				rm.setPower(1);
				telemetry.addData("Leg 2", runtime.seconds());
				telemetry.update();
			}
		}
		else {
			double[][] operations = {
				{-1.0,1.0}, // vectors
				{1.0,1.0},
				{-1.0,1.0},
				{-1.0,-1.0},
				{1.0,-1.0}
			};
			//mode Aurelien
			for(int i = 0; i<operations.length; i++){
				double vec = operations[i];
				double x = vec[0];
				double y = vec[1];
				double total_dist = (double) math.sqrt(Math.pow(y,2)+Math.pow(x,2));
				double time = time_for_dist(speed, time);
				double a = (-y+x)/Math.pow(2,1/2);
				double b = (-y-x)/Math.pow(2,1/2);
				double vmean = (Math.abs(a)+Math.abs(b))/2;
				lmvalue = (a/vmean);
				rmvalue = (b/vmean);
				runtime.reset();
				while (opModeIsActive() && (runtime.seconds() <= time)) {
					lm.setPower(lmvalue);
					rm.setPower(rmvalue);
					telemetry.addData("Runtime Seconds", runtime.seconds());
					telemetry.addData("current_operation",operations[i]);
					telemetry.addData("current_op_id",i);
					telemetry.update();
				}
			}
		}
		// run until the end of the match (driver presses STOP

	}

}

