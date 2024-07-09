package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.robot.Robot;

import java.text.spi.DecimalFormatSymbolsProvider;

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

@TeleOp(name = "FGC2024", group = "WeRobot")
public class FGC_2024 extends LinearOpMode {


    private DcMotorEx rm;
    private DcMotorEx lm;
    private DcMotorEx elv1;
    //private DcMotorEx elv2;
    //private DcMotorEx elv3;

    @Override
    public void runOpMode() throws InterruptedException {


        telemetry.addData("Status"," Initialized");
        telemetry.update();

        lm = hardwareMap.get(DcMotorEx.class, "lm");
        rm = hardwareMap.get(DcMotorEx.class, "rm");
        rm.setDirection(DcMotor.Direction.REVERSE);
        //lm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //rm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        elv1 = hardwareMap.get(DcMotorEx.class, "elv1");
        elv1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //elv2 = hardwareMap.get(DcMotorEx.class, "elv2");
        //elv2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //elv3 = hardwareMap.get(DcMotorEx.class, "elv3");
        //elv3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        boolean already_up = false;
        boolean already_down = false;
        int hauteur = 0;

        waitForStart();
        while(opModeIsActive()){
            float x = gamepad1.left_stick_x; // abscisse joystick gauche
            double y = gamepad1.left_stick_y; // ordonnées joystick gauche
            double lpower = 0.0; //puissance moteur gauche
            double rpower = 0.0; //puissance moteur droit

            lpower = ((1-x)*Math.signum(y))/1.5;
            rpower = ((1+x)*Math.signum(y))/1.5;

            if ( Math.abs(x)==1){
                lpower = 0.75*Math.signum(x);
                rpower = -lpower;
            }
            
            if(gamepad1.right_trigger == 0){
                lpower /=3;
                rpower /=3;
            }else{
                lpower = lpower*gamepad1.right_trigger;
                rpower = rpower*gamepad1.right_trigger;
            }

            if (gamepad1.dpad_up && !already_up && i<3){

                elv1.setVelocity(250);
                // elv2.setVelocity(250);
                // elv3.setVelocity(250);
                elv1.setTargetPosition(elv1.getCurrentPosition()+10);
                // elv2.setTargetPosition(elv2.getCurrentPosition()+15);
                // elv3.setTargetPosition(elv3.getCurrentPosition()+160);
                elv1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // elv2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // elv3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hauteur += 1;
                
                already_up = !already_up;

            }else{
                already_up = false;
            }

            if (gamepad1.dpad_down && !already_up && i>0){

                elv1.setVelocity(250);
                elv2.setVelocity(250);
                elv3.setVelocity(250);
                elv1.setTargetPosition(elv1.getCurrentPosition()-10);
                elv2.setTargetPosition(elv2.getCurrentPosition()-15);
                elv3.setTargetPosition(elv3.getCurrentPosition()-160);
                elv1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elv2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elv3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hauteur -= 1;
                
                already_down = !already_down;

            }else{
                already_down = false;
            }
            
            
            rm.setPower(rpower);
            lm.setPower(lpower);
            telemetry.addData("r", rpower);
            telemetry.addData("l", lpower);
            telemetry.addData("compteur", hauteur);
            telemetry.update();
        }
	}
}
