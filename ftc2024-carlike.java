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

//test
@TeleOp(name = "WeRobot: FTC2024 Carlike", group = "WeRobot")
public class Werobot_FTC2024_carlike extends LinearOpMode {

    // Le moteur de droite
    private DcMotor rm;

    // Le moteur de gauche
    private DcMotor lm;

    private DcMotor moissoneuse;

    private DcMotorEx lmelevator;

    private DcMotorEx rmelevator;

    private DcMotorEx box;

    private DcMotorEx rotation;

    private IMU imu;

    /*
     * La fonction pour faire des exponentielles spécifiques
     * 
     * @param double t => le nombre dont on veut faire l'exponentielle
     * 
     * @return double une_exponentielle_très_spéciale_de_t
     */
    private double helloexp(double t) {
        return (Math.exp(5 * t) - 1) / (Math.exp(5) - 1);
    }

    private void tele(String name, String data) {
        telemetry.addData(name, data);
        telemetry.update();
    }

    /*
     * La fonction du thread principal
     */
    @Override
    public void runOpMode() throws InterruptedException {
        // l'axe x du joystick gauche de la manette
        float x;

        // l'axe y du joystick gauche de la manette

        double y;

        // variation 1 du left trigger

        double t;

        // variation 2 du left trigger

        double t2;

        // variation 3 du left trigger

        double t3;

        // le mode du robot

        String mode = "elina";

        // b est il déjà préssé?

        boolean already_b = false;

        boolean already_a = false;

        boolean already_x = false;
        
        boolean already_y = false;

        boolean already_up = false;

        boolean already_down = false;
        
        boolean sinking = false;

        telemetry.addData("Status", "Initialized");

        // telemetry.update();

        lm = hardwareMap.get(DcMotor.class, "blm");

        rm = hardwareMap.get(DcMotor.class, "brm");

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
        box.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        box.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        // rotation positions: 20° pos initiale par rapport au sol
        rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotation.setVelocity(700, AngleUnit.DEGREES);
        rotation.setTargetPosition(40*288/360);
        
        
        
        
        imu.initialize(

                // paramètres de l'imu
                new IMU.Parameters(
                        // orientation initiale du robot
                        new RevHubOrientationOnRobot(
                                //logo vers le haut
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                //usb vers l'avant
                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        // réinitialisation du yaw de l'imu
        imu.resetYaw();
        // telemetry.addData("Mode", "calibrating...");
        // telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        // while (!isStopRequested() && !imu.isGyroCalibrated())
        // {
        // sleep(50);
        // idle();
        // }

        /*
         * ajout de la donnée en "mode": "en attente de démarrage" sur telemetry
         */
        telemetry.addData("Mode", "waiting for start");
        // telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        /*
         * mise à jour de la telemetry
         */
        telemetry.update();
        /*
         * en attente du démarrage
         */
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("rotation target Pos", rotation.getTargetPosition());
            telemetry.addData("rotation Pos", rotation.getCurrentPosition());
            telemetry.addData("rotation velocity", rotation.getVelocity());
            telemetry.update();
        }
        /*
         * le robot a démarré, le tant que le robot est activé et donc qu'il n'a pas été
         * stoppé:
         */
        while (opModeIsActive()) {
            /*
             * définition de {@link x} sur la valeur de x du joystick gauche du gamepad 1
             */
            x = gamepad1.left_stick_x;
            /*
             * définition de {@link y} sur la valeur de y du joystick gauche du gamepad 1
             */
            y = gamepad1.left_stick_y;
            /*
             * définition de {@link t} sur la valeur du trigger droit du gamepad 1
             */
            t = gamepad1.right_trigger;
            /*
             * définition de {@link t2} par utilisation de la fonction {@link helloexp} sur
             * {@link t}
             */
            t2 = helloexp(t);
            /*
             * définition de {@link t3} par utilisation de la fonction {@link helloexp} sur
             * la norme du vecteur du joystick gauche du gamepad 1 (racine carrée de {@link
             * x} au carré plus {@link y} au carré
             */
            t3 = helloexp(Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)));
            /*
             * ajout de la donnée "statut":"entrain de courrir" sur telemetry
             */
            telemetry.addData("Status", "Running");
            /*
             * si le bouton a du gamepad 1 est appuyé et {already_a} est faux
             */
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
                tele("mode", "normal");
            } else if (mode == "tank") {
                lpower = -y;
                rpower = gamepad1.right_stick_y;
                tele("mode", "tank");*/
            if (mode == "essaifranck") {
                double a = (-y + x) / Math.pow(2, 1 / 2);
                double b = (-y - x) / Math.pow(2, 1 / 2);
                double vmean = (Math.abs(a) + Math.abs(b)) / 2;
                lpower = (a / vmean) * t2;
                rpower = (b / vmean) * t2;
                tele("mode", "essai Franck");
            } else if (mode == "elina") {
                double a = (-y + x) / Math.pow(2, 1 / 2);
                double b = (-y - x) / Math.pow(2, 1 / 2);
                double vmean = (Math.abs(a) + Math.abs(b)) / 2;
                lpower = (a / vmean) * t3;
                rpower = (b / vmean) * t3;
                tele("mode", "Elina");
            }
            if (gamepad1.left_trigger>0.1) {
                lpower /= 3;
                rpower /= 3;
            }
            lm.setPower(lpower);
            rm.setPower(rpower);

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
            
            if (gamepad1.x && !already_x){
                box.setVelocity(200);
                int targetPos = 0;
                if(gamepad1.right_bumper){
                    targetPos = 0;
                }
                box.setTargetPosition(targetPos);
                box.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                already_x = !already_x;
            } else if(!gamepad1.x && already_x){
                already_x = false;
            }
            
            if (gamepad1.y && already_y){
                rotation.setVelocity(450);
                int targetPos = 0;
                if (gamepad1.right_bumper){
                    targetPos = 0;
                }else if (gamepad1.left_bumper){
                    targetPos = 0;
                }
                rotation.setTargetPosition(targetPos);
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
            // Create an object to receive the IMU angles
            YawPitchRollAngles robotOrientation;
            robotOrientation = imu.getRobotYawPitchRollAngles();

            // Now use these simple methods to extract each angle
            // (Java type double) from the object you just created:
            double Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
            double Pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
            double Roll = robotOrientation.getRoll(AngleUnit.DEGREES);
            telemetry.addData("yaw", Yaw);

            telemetry.update();
        }
    }
}
