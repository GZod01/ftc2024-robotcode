package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
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

@TeleOp(name = "WeRobot: FTC2024 NEW!!! Carlike", group = "WeRobot")
public class WEROBOT_FTC2024_New_carlike extends LinearOpMode {

    private DcMotor rm;
    private DcMotor lm;
    private DcMotor moissoneuse;
    private DcMotorEx lmelevator;
    private DcMotorEx rmelevator;
    private DcMotorEx box;
    private DcMotorEx rotation;
    private ElapsedTime     runtime = new ElapsedTime();

    /*La fonction pour faire des exponentielles spécifiques
     * @param double t => le nombre dont on veut faire l'exponentielle
     * @return double une_exponentielle_très_spéciale_de_t*/
    private double helloexp(double t) {
	return (Math.exp(5 * t) - 1) / (Math.exp(5) - 1);
    }

    //La fonction du thread principal
    @Override
    public void runOpMode() throws InterruptedException {

	double boxRot;
	int signeBR;

	float x;
	double y;

	double t;
	double t2;
	double t3;

	String mode = "elina";

	boolean already_b = false;
	boolean already_a = false;
	boolean already_x = false;
	boolean already_y = false;
	boolean already_up = false;
	boolean already_down = false;
	boolean already_ps = false;

	boolean sinking = false;
	boolean manualMode = false;
	boolean firstLaunch = true;

	telemetry.addData("Status", "Initialized");

	lm = hardwareMap.get(DcMotor.class, "blm");

	rm = hardwareMap.get(DcMotor.class, "brm");
	rm.setDirection(DcMotor.Direction.REVERSE);

	moissoneuse = hardwareMap.get(DcMotor.class, "moissonneuse");

	lmelevator = hardwareMap.get(DcMotorEx.class, "ltrselv");
	lmelevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	lmelevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

	rmelevator = hardwareMap.get(DcMotorEx.class, "rtrselv");
	rmelevator.setDirection(DcMotor.Direction.REVERSE);
	rmelevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	rmelevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

	rotation = hardwareMap.get(DcMotorEx.class, "elvRot");
	rotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	rotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

	box = hardwareMap.get(DcMotorEx.class, "boxRot");
	box.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	// box.setPositionPIDFCoefficients(5.0);

	// rotation positions: 20° pos initiale par rapport au sol
	// while (runtime.seconds()<0.5){
	//     rotation.setPower(0.5);
	// }

	telemetry.addData("Mode", "waiting for start");

	// rotation.setVelocity(1700);
	// rotation.setTargetPosition(-1000);
	// rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);

	telemetry.addData("rotation target Pos", rotation.getTargetPosition());
	telemetry.addData("rotation Pos", rotation.getCurrentPosition());
	telemetry.update();

	waitForStart();
	rotation.setVelocity(200);
	rotation.setTargetPosition(0);
	rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	while (opModeIsActive()) {

	    x = gamepad1.left_stick_x;
	    y = gamepad1.left_stick_y;

	    /* définition de {@link t} sur la valeur du trigger droit du gamepad 1*/
	    t = gamepad1.right_trigger;

	    /* définition de {@link t2} par utilisation de la fonction {@link helloexp} sur
	     * {@link t}*/
	    t2 = helloexp(t);

	    /* définition de {@link t3} par utilisation de la fonction {@link helloexp} sur
	     * la norme du vecteur du joystick gauche du gamepad 1 (racine carrée de {@link
	     * x} au carré plus {@link y} au carré*/
	    t3 = helloexp(Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)));

	    telemetry.addData("Status", "Running");
	    // si le bouton a du gamepad 1 est appuyé et {already_a} est faux
	    /*if (gamepad1.a && !already_a) {
	      if (mode == "normal") {
	      mode = "tank";
	      } else if (mode == "tank") {
	      mode = "essaifranck";
	      } else if (mode == "essaifranck") {
	      mode = "elina";
	      } else {
	      mode = "elina";
	      }
	      already_a = true;
	      }*/

	    boxRot = gamepad1.right_stick_x;
	    // if (boxRot <= 0){
	    //         signeBR = -1;
	    //     }
	    //     else{
	    //         signeBR=1;
	    //     }


	    // if (Math.abs(boxRot) < 0.1){
	    //     boxRot = 0.4*signeBR;
	    // }
	    // else {
	    //     boxRot = 0.4*signeBR + boxRot/1.67;
	    //     if (boxRot*signeBR > 0.9){
	    //         boxRot = signeBR;
	    //     }
	    // }

	    box.setPower(boxRot);



	    if (!gamepad1.a && already_a) {
		already_a = false;
	    }
	    double lpower = 0.0;
	    double rpower = 0.0;
	    /*if (mode == "normal") {
	      double ysign = y > 0 ? 1.0 : (y < 0 ? -1.0 : 0.0);
	      double xsign = x > 0 ? 1.0 : (x < 0 ? -1.0 : 0.0);
	      lpower = -ysign * t + (xsign - 2 * x) * t;
	      rpower = ysign * t + (xsign - 2 * x) * t;
	      } else if (mode == "tank") {
	      lpower = -y;
	      rpower = gamepad1.right_stick_y;*/
	    if (mode == "essaifranck") {
		double a = (-y + x) / Math.pow(2, 1 / 2);
		double b = (-y - x) / Math.pow(2, 1 / 2);
		double vmean = (Math.abs(a) + Math.abs(b)) / 2;
		lpower = (a / vmean) * t2;
		rpower = (b / vmean) * t2;
	    } else if (mode == "elina") {
		double a = (-y + x) / Math.pow(2, 1 / 2);
		double b = (-y - x) / Math.pow(2, 1 / 2);
		double vmean = (Math.abs(a) + Math.abs(b)) / 2;
		lpower = (a / vmean) * t3;
		rpower = (b / vmean) * t3;
	    }
	    if (gamepad1.left_trigger>0.1) {
		lpower /= 3;
		rpower /= 3;
	    }
	    lm.setPower(lpower);
	    rm.setPower(rpower);

	    // activation moissonneuse
	    if (gamepad1.b && !already_b) {
		double moissoneuseSpeed = 1.0;
		if (gamepad1.right_bumper){
		    moissoneuseSpeed = -1.0;
		}
		already_b = !already_b;
		if (moissoneuse.getPower() == moissoneuseSpeed) {
		    moissoneuse.setPower(0);
		} else {
		    moissoneuse.setPower(moissoneuseSpeed);
		}
	    }
	    if (!gamepad1.b && already_b) {
		already_b = false;
	    }

	    //activation elevateur
	    if (sinking && Math.abs(lmelevator.getCurrentPosition()-90)<=5 && Math.abs(rmelevator.getCurrentPosition()-90)<=5){
		lmelevator.setVelocity(100);
		rmelevator.setVelocity(100);
		lmelevator.setTargetPosition(0);
		rmelevator.setTargetPosition(0);
		lmelevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		rmelevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	    }
	    if ((gamepad1.dpad_up && !already_up)^(gamepad1.dpad_down && !already_down)){
		lmelevator.setVelocity(600);
		rmelevator.setVelocity(600);
		Long targetPosLong = (Long) Math.round(288*3.4);
		int targetPos = targetPosLong.intValue();
		if (gamepad1.dpad_down){
		    targetPos = 90;
		    already_down = true;
		    sinking = true;
		}else{
		    already_up = true;
		    sinking = false;
		}
		lmelevator.setTargetPosition(targetPos);
		rmelevator.setTargetPosition(targetPos);    
		lmelevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		rmelevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

	    }else if (!gamepad1.dpad_up && already_up){
		already_up = false;
	    }else if (!gamepad1.dpad_down && already_down){
		already_down = false;
	    }

	    if (gamepad1.ps && !already_ps){
		manualMode = !manualMode;
		already_ps = true;
	    } else if (!gamepad1.ps && already_ps){
		already_ps = false;
	    }            

	    // commentaires supprimés dans le robot
	    // if (gamepad1.x && !already_x){

	    //     int targetPos = 0;
	    //     if(gamepad1.right_bumper){
	    //         targetPos = -97;
	    //     }

	    //     // while (Math.abs(box.getCurrentPosition()-targetPos) > 30)
	    //     // {
	    //     //     box.setVelocity(100);
	    //     //     box.setTargetPosition(targetPos);
	    //     //     box.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	    //     // }



	    //     box.setVelocity(50);
	    //     box.setTargetPosition(targetPos);
	    //     box.setMode(DcMotor.RunMode.RUN_TO_POSITION);

	    //     already_x = !already_x;
	    // } else if(!gamepad1.x && already_x){
	    //     already_x = false;
	    // }

	    // if (Math.abs(box.getCurrentPosition() - box.getTargetPosition()) < 20) {
	    //     box.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
	    //     box.setPower(-0.2);
	    //     telemetry.addData("ModeChanged","without encoders");
	    // }



	    // if (gamepad1.y && already_y){
	    //     rotation.setVelocity(200);
	    //     // int targetPos = 0;
	    //     // if (gamepad1.right_bumper){
	    //     //     targetPos = 0;
	    //     // }else if (gamepad1.left_bumper){
	    //     //     targetPos = 0;
	    //     // }
	    //     // rotation.setTargetPosition(targetPos);
	    //     // rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);

	    //     int pos = rotation.getCurrentPosition();
	    //     if (gamepad1.right_bumper){
	    //         rotation.setTargetPosition(pos + 100);
	    //     }else if (gamepad1.left_bumper){
	    //         rotation.setTargetPosition(pos - 100);
	    //     }
	    //     rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	    // }


	    // activation rotation
	    if (manualMode){
		gamepad1.setLedColor(255,0,0,10);
		lmelevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		rmelevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		rotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		if (gamepad1.dpad_up){
		    rmelevator.setPower(0.3);
		    lmelevator.setPower(0.3);
		} else if (gamepad1.dpad_down){
		    lmelevator.setPower(-0.3);
		    rmelevator.setPower(-0.3);
		} else if (gamepad1.y){
		    double power = -0.3;
		    if (gamepad1.right_bumper){
			power = -power;
		    }
		    rotation.setPower(power);
		}  else {
		    lmelevator.setPower(0);
		    rmelevator.setPower(0);
		    lmelevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		    rmelevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		    rotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		}

	    }
	    else{
		gamepad1.setLedColor(0,0,255,10);
	    }

	    if (!gamepad1.y && already_y && !manualMode) {
		already_y = false;
	    }
	    if (gamepad1.y && !already_y && !manualMode){
		already_y = true;
		int pos = rotation.getCurrentPosition();
		rotation.setVelocity(200);
		if (gamepad1.right_bumper){
		    // rotation.setTargetPosition(pos - 25);
		    rotation.setTargetPosition(-100); // vertical si pos origine = 0    
		} else if (gamepad1.left_bumper){
		    // rotation.setTargetPosition(pos + 25);
		    rotation.setTargetPosition(1000); //position basse
		} else {
		    rotation.setTargetPosition(0);
		}
		rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	    }

	    if (gamepad1.right_bumper && gamepad1.left_bumper){
		// launch the plane
	    }

	    telemetry.addData("x", x);
	    telemetry.addData("y", y);
	    telemetry.addData("lpow", lpower);
	    telemetry.addData("rpow", rpower);
	    telemetry.addData("ltrigg", t);
	    telemetry.addData("t2", t2);
	    telemetry.addData("manual mode", manualMode);
	    telemetry.addData("rotation power",boxRot);
	    telemetry.addData("mode manuel", manualMode);
	    telemetry.addData("Position elevateur l", lmelevator.getCurrentPosition());
	    telemetry.addData("Position elevateur r", rmelevator.getCurrentPosition());
	    telemetry.addData("Position rotation",rotation.getCurrentPosition());
	    telemetry.addData("Position box",box.getCurrentPosition());
	    telemetry.addData("box velocity",rotation.getVelocity());
	    telemetry.update();
	      }
	}
    }
