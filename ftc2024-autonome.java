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

import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


@Autonomous

public class ftc2024_autonome extends LinearOpMode {
    private DcMotor rm;
    private DcMotor lm;
    private IMU imu;
    private ElapsedTime     runtime = new ElapsedTime();
    YawPitchRollAngles robotOrientation;
    

    public double time_for_dist(double speed, double dist){
        return (double) (dist/speed);
    }
    
    /*
     * @param double angle2go => degrees
     */
    public double runPid(double angle2go){
        robotOrientation = imu.getRobotYawPitchRollAngles();
	angle2go = (angle2go*180)/Math.PI
	double yaw = robotOrientation.getYaw(AngleUnit.RADIANS);
	double lastPos;
	// command -1.0<command<1.0
	double command = (angle2go-yaw)/2*Math.PI;
	while (opModeIsActive()){
	    yaw = robotOrientation.getYaw(AngleUnit.RADIANS);
	    double currentPos = yaw;
	    double error = currentPos-lastPos;
	    lastPos = currentPos;
	    command += P * error;
	    lm.setPower(command);
	    rm.setPower(-command);
	}

    }

    @Override
    public void runOpMode() {
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
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        double tour_par_minute = 300.0;
        double wheel_width = 9.0e-2;
        double wheel_rayon = (wheel_width)/2;
        double wheel_perimeter = wheel_rayon*2*Math.PI;
        double speed = (tour_par_minute/60)*wheel_perimeter;//dist per second
        boolean mode = true;
        
        robotOrientation = imu.getRobotYawPitchRollAngles();
        
        double Yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);
        double Pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
        double Roll  = robotOrientation.getRoll(AngleUnit.DEGREES);
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runtime.reset();
        if (mode){
            //mode Elina
            while (opModeIsActive() && Yaw <= 90.0) {
                lm.setPower(0.5);
                rm.setPower(-0.5);
                Yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);
                telemetry.addData("Leg 1", runtime.seconds());
                telemetry.update();
            }
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() <= 121.92e-2/speed)) {
                lm.setPower(1);
                rm.setPower(1);
                telemetry.addData("Leg 2", runtime.seconds());
                telemetry.update();
            }
        }
        else{
	    while(opModeIsActive()){
		Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
		if(Math.abs(Yaw-90.0)<=0.01){
		    break;
		}
		else if((Yaw - 90.0) <0){
		    lm.setPower((Math.abs(Yaw-90.0)/90)*0.5);
		    rm.setPower(-(Math.abs(Yaw-90.0)/90)*0.5);
		}
		else{
		    rm.setPower((Math.abs(Yaw-90.0)/90)*0.5);
		    lm.setPower(-(Math.abs(Yaw-90.0)/90)*0.5);
		}
	    }
	}

	if(false){
            double[][] operations = {
                {-1.0,1.0}, // vectors
                {1.0,1.0},
                {-1.0,1.0},
                {-1.0,-1.0},
                {1.0,-1.0}
            };
            //mode Aurelien
            for(int i = 0; i<operations.length; i++){
                double[] vec = operations[i];
                double x = vec[0];
                double y = vec[1];
                double total_dist = (double) Math.sqrt(Math.pow(y,2)+Math.pow(x,2));
                double time = time_for_dist(speed, total_dist);
                double a = (-y+x)/Math.pow(2,1/2);
                double b = (-y-x)/Math.pow(2,1/2);
                double vmean = (Math.abs(a)+Math.abs(b))/2;
                double lmvalue = (a/vmean);
                double rmvalue = (b/vmean);
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

        // Now use these simple methods to extract each angle
        // (Java type double) from the object you just created:
        
        telemetry.addData("yaw",Yaw);

        telemetry.update();
        // run until the end of the match (driver presses STOP

    }

}

