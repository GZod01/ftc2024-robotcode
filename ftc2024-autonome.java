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
                rightMotor.setPower(1);
                telemetry.addData("Leg 1", runtime.seconds());
                telemetry.update();
            }
        }
        else {
	    double[][] operations = {
		    {3.0,-1.0,1.0} // operation 1: 3 sec , lm=-1 , rm = 1
	    };
            //mode Aurelien
	    for(int i = 0; i<operations.length; i++){
		    double time = operations[i][0];
		    double lmvalue = operations[i][1];
		    double rmvalue = operations[i][2];
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
		// run until the end of the match (driver presses STOP)

		
		


		

	}

}
