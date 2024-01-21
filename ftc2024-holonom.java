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
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


@TeleOp(name="WeRobot: FTC2024", group="WeRobot")
public class Werobot_FTC2024 extends LinearOpMode {
        private DcMotor flm;
        private DcMotor frm;
        private DcMotor brm;
        private DcMotor blm;
	private IMU imu;
	public double helloexp(double t){
		return (Math.exp(5*t)-1)/(Math.exp(5)-1);
	}
        @Override
        public void runOpMode() throws InterruptedException {
                float x;
                double y;
		String mode="normal";
		boolean already_a = false;
                int[] fr_angle = {1,1};
                int[] fl_angle = {1,-1};
                int[] bl_angle = {-1,-1};
                int[] br_angle = {-1, 1};
                int[][] motors_angles = {fl_angle,
                fr_angle,
                bl_angle,
                br_angle
                };
                telemetry.addData("Status", "Initialized");
                telemetry.update();
                flm = hardwareMap.get(DcMotor.class, "flm");
                frm = hardwareMap.get(DcMotor.class, "frm");
                blm = hardwareMap.get(DcMotor.class, "blm");
                brm = hardwareMap.get(DcMotor.class, "brm");
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
		telemetry.addData("helloworld","hello");
		telemetry.update();
                waitForStart();
                
                
                while (opModeIsActive()) {
			x = gamepad1.left_stick_x;
			y = gamepad1.left_stick_y;
                        telemetry.addData("Status", "Running");
			YawPitchRollAngles robotOrientation;
			robotOrientation = imu.getRobotYawPitchRollAngles();

			// Now use these simple methods to extract each angle
			// (Java type double) from the object you just created:
			double Yaw   = robotOrientation.getYaw(AngleUnit.RADIANS);
			double Pitch = robotOrientation.getPitch(AngleUnit.RADIANS);
			double Roll  = robotOrientation.getRoll(AngleUnit.RADIANS);
                        double[] motors_values = new double[4];
                        telemetry.addData("left_stick_x", x);
                        telemetry.addData("left_stick_y", y);
                        double[] joystick_vector = {(double) x,y}; 
                        double joystick_norm = Math.pow(
                                (
                                        (
                                                Math.pow(joystick_vector[0],2)
                                        )+(
                                                Math.pow(joystick_vector[1],2)
                                        )
                                ),(1.0/2.0)
                        );
			joystick_norm = helloexp(joystick_norm);
			if(gamepad1.a && !already_a){
				if(mode=="normal"){
					mode="IMU";
				}else{
					mode="normal";
				}
			}
			if(!gamepad1.a && already_a){
				already_a = false;
			}
                        for(int i = 0; i<motors_angles.length; i++){
                                int[] cur_motor = motors_angles[i];
                                double cur_motor_norm = Math.pow(((Math.pow(cur_motor[0],2))+(Math.pow(cur_motor[1],2))),(1.0/2.0));
                                double joystick_angle = Math.atan2(joystick_vector[0],joystick_vector[1]);
                                double cur_motor_angle = Math.atan2(cur_motor[0],cur_motor[1]);
                                double diff_angle = joystick_angle - cur_motor_angle+(Math.PI/4);
				if (mode=="normal"){
					diff_angle-=Yaw;
				}
                                motors_values[i] =  (cur_motor_norm*joystick_norm*Math.cos(diff_angle));
                                motors_values[i] = (motors_values[i]+gamepad1.right_stick_x)/(Math.abs(gamepad1.right_stick_x)+1);
                                motors_values[i]= motors_values[i]/Math.sqrt(2);
                                if(!(gamepad1.left_trigger > 0.3)){
                                        motors_values[i]= (motors_values[i])/(3*Math.sqrt(2));
                                }
                                telemetry.addData("joystick_norm of "+i+" ",joystick_norm);
                                telemetry.addData("motors_values["+i+"]",motors_values[i]);
                                
                        }
                        
                        flm.setPower(motors_values[0]);
                        frm.setPower(motors_values[1]);
                        blm.setPower(motors_values[2]);
                        brm.setPower(motors_values[3]);
			telemetry.addData("yaw",Yaw);
			telemetry.addData("Mode",mode);
                        telemetry.update();
                }
        }
}
