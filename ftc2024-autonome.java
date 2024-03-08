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
<<<<<<< HEAD
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
	angle2go = (angle2go*180)/Math.PI;
	double P = 0.5;
	while (opModeIsActive()){
	    yaw = robotOrientation.getYaw(AngleUnit.RADIANS);
	    double currentPos = yaw;
	    double error = yaw-currentpos;
	    double command = (P * error)/Math.PI;
	    command = clamp(command,.6); 
	    lm.setPower(command);
	    rm.setPower(-command);
	}

    }
    double clamp(double value, double max) {
        return Math.max(-max, Math.min(max, value));
    }
=======
>>>>>>> 7cd7be1f14eec5f1be12b9f60543771de58ca782

    @Override

    public void runOpMode() {
        lm = hardwareMap.get (DcMotor.class, "blm");
        rm = hardwareMap.get (DcMotor.class, "brm");

        rm.setDirection(DcMotor.Direction.REVERSE);
        
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
<<<<<<< HEAD

        rm.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        double tour_par_minute = 300.0;
        double wheel_width = 9.0e-2;
        double wheel_rayon = (wheel_width)/2;
        double wheel_perimeter = wheel_rayon*2*Math.PI;
        double speed = (tour_par_minute/60)*wheel_perimeter;//dist per second
        boolean mode = true;
        
=======
        YawPitchRollAngles robotOrientation;
>>>>>>> 7cd7be1f14eec5f1be12b9f60543771de58ca782
        robotOrientation = imu.getRobotYawPitchRollAngles();
        double Yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);
        double yaw_sortie = 0.0;

        waitForStart();

<<<<<<< HEAD
        runtime.reset();
        if (mode){
            //mode Elina
            while (opModeIsActive() && Yaw <= 90.0) {
                lm.setPower(0.5);
                rm.setPower(-0.5);
				robotOrientation = imu.getRobotYawPitchRollAngles();
                Yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);
                telemetry.addData("Yaw : ", Yaw);
                telemetry.update();
				yaw_sortie = Yaw;
            }
			telemetry.addData("yaw_sortie", yaw_sortie)
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() <= 121.92e-2/speed)) {
                lm.setPower(0.1);
                rm.setPower(0.1);
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

                    
=======
        while (opModeIsActive()){
            double [] lm_p = {0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1};
            double [] rm_p = {-0.1,-0.2,-0.3,-0.4,-0.5,-0.6,-0.7,-0.8,-0.9,-1};
            double [] x = new double[lm_p.length];
            for(int i = 0; i< 9; i++){
                while (opModeIsActive() && Yaw < 90){
                    lm.setPower(lm_p[i]);
                    rm.setPower(rm_p[i]);
                    robotOrientation = imu.getRobotYawPitchRollAngles();
                    Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
                    telemetry.addData("Yaw ", Yaw);
>>>>>>> 7cd7be1f14eec5f1be12b9f60543771de58ca782
                    telemetry.update();
                    yaw_sortie = Yaw;
                }
                telemetry.addData("Yaw sortie", yaw_sortie);
                telemetry.update();
                Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
                x [i]= Yaw - yaw_sortie;
                
                imu.resetYaw();
                Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
                telemetry.addData("Yaw", Yaw);
                telemetry.update();
            }
            while (opModeIsActive()){
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
}