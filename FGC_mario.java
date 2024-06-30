package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
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

@TeleOp(name = "FGCmario", group = "WeRobot")
public class FGC_mario {
    @Override
    public void runOpMode() throws InterruptedException {


        telemetry.addData("Status"," Initialized");
        telemetry.update();

        lm = hardwareMap.get(DcMotorEx.class, "lm");
        rm = hardwareMap.get(DcMotorEx.class, "rm");
        rm.setDirection(DcMotor.Direction.REVERSE);
        //lm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //rm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();
        while(opModeIsActive()){
            float x = gamepad1.left_stick_x; // abscisse joystick gauche
            double y = gamepad1.left_stick_y; // ordonnées joystick gauche
            double a = gamepad1.right_trigger-gamepad1.left_trigger;
            double lpower = (1-x)/1.5; //puissance moteur gauche
            double rpower = (1+x)/1.5; //puissance moteur droit

            if (a==0){
                lpower*=Math.abs(y)/3;
                rpower*=Math.abs(y)/3;
            }else{
                lpower *= a;
                rpower *= a;
            }

            if ( Math.abs(x)==1){
                lpower = 0.75*Math.signum(x);
                rpower = -lpower;
            }

            rm.setPower(rpower);
            lm.setPower(lpower);
            telemetry.addData("l", lpower);
            telemetry.addData("r", rpower);
            telemetry.update();
        }
    }
}
