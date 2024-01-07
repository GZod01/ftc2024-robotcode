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
		rm.setDirection(DcMotorSimple.Direction.REVERSE);
                waitForStart();
                
                
                while (opModeIsActive()) {
                        x = gamepad1.left_stick_x;
                        y = gamepad1.left_stick_y;
                        t= gamepad1.right_trigger;
			t2 = helloexp(t);
                        t3 = helloexp(Math.sqrt(x^2+y^2))
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
                                lpower = (a/vmean)*t3
                                rpower = (b/vmean)*t3
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
                        telemetry.update();
                }
        }
}
