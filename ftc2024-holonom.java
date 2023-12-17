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


@TeleOp(name="WeRobot: FTC2024", group="WeRobot")
public class Werobot_FTC2024 extends LinearOpMode {
        private DcMotor flm;
        private DcMotor frm;
        private DcMotor brm;
        private DcMotor blm;
        @Override
        public void runOpMode() throws InterruptedException {
                float x;
                double y;
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
                waitForStart();
                
                
                while (opModeIsActive()) {
                        telemetry.addData("Status", "Running");
                        double[] motors_values = new double[4];
                        telemetry.addData("left_stick_x", gamepad1.left_stick_x);
                        telemetry.addData("left_stick_y", gamepad1.left_stick_y);
                        double[] joystick_vector = {(double) gamepad1.left_stick_x,gamepad1.left_stick_y}; 
                        double joystick_norm = Math.pow(
                                (
                                        (
                                                Math.pow(joystick_vector[0],2)
                                        )+(
                                                Math.pow(joystick_vector[1],2)
                                        )
                                ),(1.0/2.0)
                        );
                        for(int i = 0; i<motors_angles.length; i++){
                                int[] cur_motor = motors_angles[i];
                                double cur_motor_norm = Math.pow(((Math.pow(cur_motor[0],2))+(Math.pow(cur_motor[1],2))),(1.0/2.0));
                                double joystick_angle = Math.atan2(joystick_vector[0],joystick_vector[1]);
                                double cur_motor_angle = Math.atan2(cur_motor[0],cur_motor[1]);
                                double diff_angle = joystick_angle - cur_motor_angle;
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
                        telemetry.update();
                }
        }
}
