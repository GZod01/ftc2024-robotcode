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


@TeleOp(name="WeRobot: FTC2024 Carlike", group="WeRobot")
public class Werobot_FTC2024_carlike extends LinearOpMode {
        private DcMotor rm;
        private DcMotor lm;
        @Override
        public void runOpMode() throws InterruptedException {
                float x;
                double y;
                String mode = "normal";
                boolean already_a = false;
                telemetry.addData("Status", "Initialized");
                telemetry.update();
                lm = hardwareMap.get(DcMotor.class, "blm");
                rm = hardwareMap.get(DcMotor.class, "brm");
                waitForStart();
                
                
                while (opModeIsActive()) {
                        telemetry.addData("Status", "Running");
                        if(gamepad1.a && !already_a){
                                if(mode=="normal"){
                                        mode="tank";
                                }else if(mode=="tank"){
					mode = "essaifranck";
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
                                double ysign = gamepad1.left_stick_y>0?1.0:(gamepad1.left_stick_y<0?-1.0:0.0);
                                double xsign = gamepad1.left_stick_x>0?1.0:(gamepad1.left_stick_x<0?-1.0:0.0);
                                lpower = -ysign * gamepad1.left_trigger + (xsign-2*gamepad1.left_stick_x)*gamepad1.left_trigger;
                                rpower = ysign * gamepad1.left_trigger + (xsign-2*gamepad1.left_stick_x)*gamepad1.left_trigger;
                        }
                        else if (mode=="tank"){
                                lpower = -gamepad1.left_stick_y;
                                rpower = gamepad1.right_stick_y;
                        }
			else if (mode=="essaifranck"){
				double a = (gamepad1.left_stick_x-gamepad1.left_stick_y)/Math.pow(2,1/2);
				double b = (gamepad1.left_stick_x+gamepad1.left_stick_y)/Math.pow(2,1/2);
                                double asqr_plus_bsqr = Math.pow(a,2)+Math.pow(b,2);
                                lpower = (a/asqr_plus_bsqr)*Math.pow(gamepad1.left_trigger,1/2);
                                rpower = (b/asqr_plus_bsqr)*Math.pow(gamepad1.left_trigger,1/2);
			}
                        if(!(gamepad1.left_bumper)){
                                lpower/=3;
                                rpower/=3;
                        }
                        lm.setPower(lpower);
                        rm.setPower(rpower);
                        telemetry.addData("ltrigg",gamepad1.left_trigger)
                        telemetry.addData("mode",mode);
                        telemetry.update();
                }
        }
}
