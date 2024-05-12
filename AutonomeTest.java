package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous

public class AutonomeTest extends LinearOpMode {
    private DcMotorEx droit;
    private DcMotorEx gauche;
    private DcMotorEx boiteG;
    private DcMotorEx boiteD;
    private ElapsedTime runtime = new ElapsedTime();
    
    @Override
    public void runOpMode(){
        boiteD = hardwareMap.get(DcMotorEx.class, "rtrselv");
        boiteD.setDirection(DcMotorSimple.Direction.REVERSE);
        boiteD.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        boiteD.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        boiteG = hardwareMap.get(DcMotorEx.class, "ltrselv");
        boiteG.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        boiteG.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        gauche = hardwareMap.get(DcMotorEx.class, "blm");
        gauche.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        gauche.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        droit = hardwareMap.get(DcMotorEx.class, "brm");
        droit.setDirection(DcMotorSimple.Direction.REVERSE);
        droit.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        droit.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
	moissonneuse = hardwareMap.get(DcMotor.class,"msn");
        
        
        waitForStart();
        
        boiteG.setTargetPosition(90);
        boiteD.setTargetPosition(90);
        while (opModeIsActive() && boiteG.getCurrentPosition()<boiteG.getTargetPosition()){
            boiteD.setVelocity(100);
            boiteD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            boiteG.setVelocity(100);
            boiteG.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addData("position elv droit", boiteD.getCurrentPosition());
            telemetry.addData("position elv gauche", boiteG.getCurrentPosition());
            telemetry.update();
        }
        gauche.setTargetPosition(700*3);
        droit.setTargetPosition(700*3);
        
        while (opModeIsActive() && gauche.getCurrentPosition()<gauche.getTargetPosition()){
            gauche.setVelocity(250);
            gauche.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            droit.setVelocity(250);
            droit.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        gauche.setVelocity(0);
        droit.setVelocity(0);
        gauche.setTargetPosition(165);
        droit.setTargetPosition(1235);
        
        while (opModeIsActive() && gauche.getCurrentPosition()>gauche.getTargetPosition()){
            gauche.setVelocity(250);
            gauche.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            droit.setVelocity(250);
            droit.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        gauche.setTargetPosition(700*3);
        droit.setTargetPosition(700*3);
        
        while (opModeIsActive() && gauche.getCurrentPosition()<gauche.getTargetPosition()){
	    moissonneuse.setPower(1);
            gauche.setVelocity(250);
            gauche.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            droit.setVelocity(250);
            droit.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
	moissonneuse.setPower(0);
        gauche.setVelocity(0);
        droit.setVelocity(0);
        
    }
}
