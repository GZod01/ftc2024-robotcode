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

@TeleOp(name = "WeRobot: FTC2024 NEW! Carlike", group = "WeRobot")
public class WEROBOT_FTC2024_New_carlike extends LinearOpMode {
    public enum RobotMode {
        ESSAIFRANCK, ELINA, NORMAL, TANK;
    }

    private DcMotorEx rm;
    private DcMotorEx lm;
    private DcMotor moissoneuse;
    private DcMotorEx lmelevator;
    private DcMotorEx rmelevator;
    private DcMotorEx box;
    private DcMotorEx rotation;
    private Servo avion;
    private ElapsedTime runtime = new ElapsedTime();
    private RobotMode mode = RobotMode.ESSAIFRANCK;

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

    public void nextMode() {
        RobotMode toNextMode;
        switch (this.mode) {
            case ESSAIFRANCK:
                toNextMode = RobotMode.ELINA;
                break;
            case ELINA:
                toNextMode = RobotMode.NORMAL;
                break;
            case NORMAL:
                toNextMode = RobotMode.TANK;
                break;
            default:
                toNextMode = RobotMode.ESSAIFRANCK;
                break;
        }
        this.mode = toNextMode;
    }
    public boolean isBetween(double elem, double mini, double maxi){
        return Math.abs(elem - (((maxi-mini)/2)+mini))<=(maxi-mini)/2;
    }
    // La fonction du thread principal
    @Override
    public void runOpMode() throws InterruptedException {

        double boxRot = 0;
        int signeBR;

        float x;
        double y;

        double t;
        double t2;
        double t3;

        boolean already_b = false;
        boolean already_a = false;
        boolean already_x = false;
        boolean already_y = false;
        boolean already_up = false;
        boolean already_down = false;
        boolean already_ps = false;
        
        boolean already_paddown = false;
        boolean already_padup = false;
        boolean already_padright = false;
        boolean already_padleft = false;
        

        boolean sinking = false;
        boolean manualMode = false;
        boolean firstLaunch = true;

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Version", "5.123");

        lm = hardwareMap.get(DcMotorEx.class, "blm");

        rm = hardwareMap.get(DcMotorEx.class, "brm");
        rm.setDirection(DcMotor.Direction.REVERSE);
        
        lm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        moissoneuse = hardwareMap.get(DcMotor.class, "moissonneuse");
        moissoneuse.setDirection(DcMotor.Direction.REVERSE);

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
        box.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        avion = hardwareMap.get(Servo.class, "avion");
        // box.setPositionPIDFCoefficients(5.0);

        // rotation positions: 20° pos initiale par rapport au sol
        // while (runtime.seconds()<0.5){
        // rotation.setPower(0.5);
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

            /* définition de {@link t} sur la valeur du trigger droit du gamepad 1 */
            t = gamepad1.right_trigger;

            /*
             * définition de {@link t2} par utilisation de la fonction {@link helloexp} sur
             * {@link t}
             */
            t2 = helloexp(t);

            /*
             * définition de {@link t3} par utilisation de la fonction {@link helloexp} sur
             * la norme du vecteur du joystick gauche du gamepad 1 (racine carrée de {@link
             * x} au carré plus {@link y} au carré)
             */
            t3 = helloexp(Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)));

            telemetry.addData("Status", "Running");
            

			if (gamepad1.right_bumper && gamepad1.left_bumper && gamepad1.a){
				avion.setPosition(0);
				continue;
			}
            // Choix mode conduite / actif en manuel et auto
            if (gamepad1.a && !already_a) {
                nextMode();
                already_a = true;
            }
            if (!gamepad1.a && already_a) {
                already_a = false;
            }
            double lpower = 0.0;
            double rpower = 0.0;
            double vmean;
            double a;
            double b;
			
            switch (mode) {

                case NORMAL:
                    double ysign = Math.signum(y);
                    double xsign = Math.signum(x);
                    lpower = ysign * t + (-xsign - 2 * x) * t;
                    rpower = ysign * t + (xsign - 2 * x) * t;
                    break;


                case TANK:
                    lpower = -y;
                    rpower = -gamepad1.right_stick_y;
                    break;

                case ESSAIFRANCK:
                    // Code ci-dessous OK
                    // lpower = (1 + x); //(a / vmean) * t2;
                    // rpower = 2-lpower;//(b / vmean) * t2;
                    // lpower*=t2*((Math.signum(y)==0)?1:-Math.signum(y)); // sigNum(0)
                    // rpower*=t2*((Math.signum(y)==0)?1:-Math.signum(y));
                    // Fin code OK
                    
                    int ltargetPos, rtargetPos;
                    ltargetPos = rtargetPos = 0;
                    
                    int lmpos = lm.getCurrentPosition();
                    int rmpos = rm.getCurrentPosition();
                    int step = 100;
                    // double rapp = step/theta;
                    double signe = Math.signum(-y);
                    signe = (signe == 0)?1.0:signe;
					if(Math.abs(x)<=0.1 && Math.abs(y)<=0.1) ltargetpos = rtargetpos = 0;
                    else if (Math.abs(x)<=0.1){
                        ltargetPos = rtargetPos = 100;
                    }else if (isBetween(x,0.1,0.9)){
                        rtargetPos = step;
                        ltargetPos = 2*step;
                    }else if (x>=0.9){
                        ltargetPos = step;
                        rtargetPos = -step;
                    }else if (x<=-0.9){
                        ltargetPos = -step;
                        rtargetPos = step;
                    }else if (isBetween(x, -0.9,-0.1)){
                        rtargetPos = 2*step;
                        ltargetPos = step;
                    }
                    ltargetPos*=signe;
                    rtargetPos*=signe;
                            
                    // if (x>0.1){
                    //     theta = Math.abs(Math.PI/2 - Math.atan2(-y,x));
                    //     theta = (theta > Math.PI/2)?(Math.PI - theta):theta;
                    //     ltargetPos = (int) (Math.floor(theta*37.0 + step)*signe);
                    //     rtargetPos = (int) (Math.floor(step)*signe);
                    // }
                    // else if (x < -0.1){
                    //     theta = Math.abs(Math.abs(Math.atan2(-y,x)) - Math.PI/2);
                    //     rtargetPos = (int) (Math.floor(theta*37.0 + step)*signe);
                    //     ltargetPos = (int) (Math.floor(step)*signe);
                    // }
                    // else {
                    //     rtargetPos = step*((int) Math.signum(-y));
                    //     ltargetPos = step*((int) Math.signum(-y));
                    // }
                    
                    telemetry.addData("ltargetPos avant cut",ltargetPos);
                    telemetry.addData("rtargetPos avant cut",rtargetPos);
                    
                    // ltargetPos = ((Math.abs(ltargetPos)> step)?(int) Math.signum(ltargetPos)*(step + Math.abs(ltargetPos)%step ):ltargetPos);
                    // rtargetPos = ((Math.abs(rtargetPos)> step)?(int) Math.signum(rtargetPos)*(step + Math.abs(rtargetPos)%step):rtargetPos);
                    
                    lm.setTargetPosition(lmpos + ltargetPos);
                    rm.setTargetPosition(rmpos + rtargetPos);
                    lm.setVelocity(2800.0*t2);
                    rm.setVelocity(2800.0*t2);
                    
                    
                    // telemetry.addData("rapp",rapp);
                    telemetry.addData("ltargetPos",ltargetPos);
                    telemetry.addData("rtargetPos",rtargetPos);
                    
                    lm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    
                    
                    
                    break;

                case ELINA:
                    a = (-y + x) / Math.pow(2, 1 / 2);
                    b = (-y - x) / Math.pow(2, 1 / 2);
                    vmean = (Math.abs(a) + Math.abs(b)) / 2;
                    lpower = (a / vmean) * t3;
                    rpower = (b / vmean) * t3;
                    break;

            }

            if (gamepad1.left_trigger > 0.1) {
                lpower /= 3;
                rpower /= 3;
            }

            // CODE OK
            // lm.setPower(lpower/1.5);
            // rm.setPower(rpower/1.5);
            // Fin code OK

            // activation moissonneuse -- actif en manuel et auto
            if (gamepad1.b && !already_b) {
                double moissoneuseSpeed = 1.0;
                if (gamepad1.right_bumper) {
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

            // activation elevateur
            if (sinking && Math.abs(lmelevator.getCurrentPosition() - 90) <= 5
                    && Math.abs(rmelevator.getCurrentPosition() - 90) <= 5) {
                lmelevator.setVelocity(100);
                rmelevator.setVelocity(100);
                lmelevator.setTargetPosition(0);
                rmelevator.setTargetPosition(0);
                lmelevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rmelevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if ((gamepad1.dpad_up && !already_up) ^ (gamepad1.dpad_down && !already_down)) {
                lmelevator.setVelocity(600);
                rmelevator.setVelocity(600);
                Long targetPosLong = (Long) Math.round(288 * 3.4);
                int targetPos = targetPosLong.intValue();
                if (gamepad1.dpad_down) {
                    targetPos = 90;
                    already_down = true;
                    sinking = true;
                } else {
                    already_up = true;
                    sinking = false;
                }
                lmelevator.setTargetPosition(targetPos);
                rmelevator.setTargetPosition(targetPos);
                lmelevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rmelevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            } else if (!gamepad1.dpad_up && already_up) {
                already_up = false;
            } else if (!gamepad1.dpad_down && already_down) {
                already_down = false;
            }

            if (gamepad1.ps && !already_ps) {
                manualMode = !manualMode;
                already_ps = true;
            } else if (!gamepad1.ps && already_ps) {
                already_ps = false;
            }

            // activation rotation
            if (manualMode) {
                gamepad1.setLedColor(1.0, 0.0, 0.0,255);
                // lmelevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                // rmelevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                
                rotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                box.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                telemetry.addData("mode","MANUEL");
                // Elevator manual mode
                if (gamepad1.dpad_up) {
                    // rmelevator.setPower(0.3);
                    // lmelevator.setPower(0.3);
                    lmelevator.setVelocity(400);
                    rmelevator.setVelocity(400);
                    int lpos = rmelevator.getCurrentPosition();
                    int rpos = lmelevator.getCurrentPosition();
                    lmelevator.setTargetPosition(lpos + 15);
                    rmelevator.setTargetPosition(rpos + 15);
                    lmelevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rmelevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                } else if (gamepad1.dpad_down) {
                    // lmelevator.setPower(-0.3);
                    // rmelevator.setPower(-0.3);
                    lmelevator.setVelocity(400);
                    rmelevator.setVelocity(400);
                    int lpos = rmelevator.getCurrentPosition();
                    int rpos = lmelevator.getCurrentPosition();
                    lmelevator.setTargetPosition(lpos - 15);
                    rmelevator.setTargetPosition(rpos - 15);
                    lmelevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rmelevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                } else if (gamepad1.y) {
                    double power = -0.3;
                    if (gamepad1.right_bumper) {
                        power = -power;
                    }
                    rotation.setPower(power);
                } else {
                    // lmelevator.setPower(0);
                    // rmelevator.setPower(0);
                    rotation.setPower(0);
                    // lmelevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    // rmelevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    rotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    
                }
                // Box manual mode
                if (gamepad1.dpad_left) {
                    box.setPower(0.3);
                    box.setPower(0.3);
                } else if (gamepad1.dpad_right) {
                    box.setPower(-0.3);
                    box.setPower(-0.3);
                } 
                else {
                    box.setPower(0);
                    box.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    
                }
                // Accrochage final
                if (gamepad1.x){
                    // lmelevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    // rmelevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lmelevator.setVelocity(600);
                    rmelevator.setVelocity(600);
                    lmelevator.setTargetPosition(40);
                    rmelevator.setTargetPosition(40);
                    lmelevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rmelevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

            } else {
                gamepad1.setLedColor(0.0, 0.0, 0.0,10);
                telemetry.addData("mode","AUTOMATIQUE");
                
                if (!gamepad1.dpad_right && already_padright) {
                    already_padright = false;
                }
                if (gamepad1.dpad_right && !already_padright) {
                    already_padright = true;
                    // POSITION INITIALE
                    rotation.setVelocity(600);
                    rotation.setTargetPosition(0);
                    
                    lmelevator.setVelocity(600);
                    lmelevator.setTargetPosition(0);
                    rmelevator.setVelocity(600);
                    rmelevator.setTargetPosition(0);
                    
                    rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lmelevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rmelevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                
                if (!gamepad1.dpad_left && already_padleft) {
                    already_padleft = false;
                }
                if (gamepad1.dpad_left && !already_padleft) {
                    already_padleft = true;
                    // POSITION ROULAGE  / Rammase Pixel dans boite
                    rotation.setVelocity(600);
                    rotation.setTargetPosition(-50);
                    
                    lmelevator.setVelocity(600);
                    lmelevator.setTargetPosition(150);
                    rmelevator.setVelocity(600);
                    rmelevator.setTargetPosition(150);
                    
                    rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lmelevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rmelevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            

                if (!gamepad1.dpad_up && already_padup) {
                    already_y = false;
                }
                if (gamepad1.dpad_up && !already_padup) {
                    already_padup = true;
                    // POSITION CHASSE-NEIGE
                    rotation.setVelocity(600);
                    rotation.setTargetPosition(110);
                    
                    lmelevator.setVelocity(600);
                    lmelevator.setTargetPosition(555);
                    rmelevator.setVelocity(600);
                    rmelevator.setTargetPosition(555);
                    
                    rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lmelevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rmelevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    
                    
                    // int pos = rotation.getCurrentPosition();
                    // rotation.setVelocity(200);
                    // if (gamepad1.right_bumper) {
                    //     // rotation.setTargetPosition(pos - 25);
                    //     rotation.setTargetPosition(-100); // vertical si pos origine = 0
                    // } else if (gamepad1.left_bumper) {
                    //     // rotation.setTargetPosition(pos + 25);
                    //     rotation.setTargetPosition(1000); // position basse
                    // } else {
                    //     rotation.setTargetPosition(0);
                    // }
                    // rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                
                if (!gamepad1.dpad_down && already_paddown) {
                    already_a = false;
                }
                if (gamepad1.dpad_down && !already_paddown) {
                    already_a = true;
                    // POSITION BASSE
                    rotation.setVelocity(600);
                    rotation.setTargetPosition(800);
                    
                    lmelevator.setVelocity(600);
                    lmelevator.setTargetPosition(0);
                    rmelevator.setVelocity(600);
                    rmelevator.setTargetPosition(0);
                    
                    rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lmelevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rmelevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            }

     

            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.addData("mode", mode);
            telemetry.addData("lpow", lpower);
            telemetry.addData("rpow", rpower);
            telemetry.addData("ltrigg", t);
            telemetry.addData("t2", t2);
            telemetry.addData("rotation power", boxRot);
            telemetry.addData("Position elevateur l", lmelevator.getCurrentPosition());
            telemetry.addData("Position elevateur r", rmelevator.getCurrentPosition());
            telemetry.addData("Position rotation", rotation.getCurrentPosition());
            telemetry.addData("Position box", box.getCurrentPosition());
            telemetry.addData("box velocity", rotation.getVelocity());
            telemetry.update();
        }
    }

}