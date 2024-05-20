package org.firstinspires.ftc.teamcode;

//import FTC2024WeRobotControl; //a tester car pas sur que ça fonctionne

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous

public class lastmatch extends LinearOpMode {

    public DcMotorEx lm;
    public DcMotorEx rm;
    public DcMotorEx lmelevator;
    public DcMotorEx rmelevator;
    public DcMotor harvestmotor;
    public IMU imu;
    public DcMotorEx rotation;
    private ElapsedTime timer;
    

    @Override
    public void runOpMode() {
        timer = new ElapsedTime();
        boolean auto = false;
        lm = hardwareMap.get(DcMotorEx.class, "blm");
        rm = hardwareMap.get(DcMotorEx.class, "brm");
        harvestmotor = hardwareMap.get(DcMotor.class, "moissonneuse");
        rotation = hardwareMap.get(DcMotorEx.class, "elvRot");
        rotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lmelevator = hardwareMap.get(DcMotorEx.class, "ltrselv");
        lmelevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lmelevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rmelevator = hardwareMap.get(DcMotorEx.class, "rtrselv");
        rmelevator.setDirection(DcMotor.Direction.REVERSE);
        rmelevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rmelevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rm.setDirection(DcMotor.Direction.REVERSE);
        rotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();
        YawPitchRollAngles robotOrientation;
        FTC2024WeRobotControl robot = new FTC2024WeRobotControl(this);

        telemetry.addData("wait for start", "");
        telemetry.update();

        waitForStart();
        telemetry.addData("started", "");
        telemetry.update();
        robotOrientation = imu.getRobotYawPitchRollAngles();

        while (opModeIsActive()) {
            if (gamepad1.a && !auto) {
                auto = true;
                break;
            }
        }
        if (opModeIsActive()) {
            double motor_speed = 1.0;
            lmelevator.setVelocity(600);
            rmelevator.setVelocity(600);
            lmelevator.setTargetPosition(200);
            rmelevator.setTargetPosition(200);
            rotation.setVelocity(600);
            rotation.setTargetPosition(40);
            lmelevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rmelevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(2000);
            
            lm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            double total_time = robot.time_for_dist(1 * robot.ground_tiles_width, Math.abs(motor_speed));
            timer.reset();
            while (opModeIsActive() && timer.seconds() < total_time) {
                lm.setPower(motor_speed);
                rm.setPower(motor_speed);
            }
            lm.setPower(0);
            rm.setPower(0);
            
            harvestmotor.setPower(-0.6);
            
            lm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            total_time = robot.time_for_dist(0.7 * robot.ground_tiles_width, Math.abs(motor_speed));
            timer.reset();
            while (opModeIsActive() && timer.seconds() < total_time) {
                lm.setPower(-motor_speed);
                rm.setPower(-motor_speed);
            }
            lm.setPower(0);
            rm.setPower(0);
            
            double angle = 90.0;
            lm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            double perimeter = Math.toRadians(angle)* 37.0/2.0;
            int targetPos = (int) Math.floor(perimeter/(9e-2*Math.PI));
            targetPos = targetPos * 20;
            rm.setTargetPosition(targetPos);
            lm.setTargetPosition(-targetPos);
            lm.setVelocity(600);
            rm.setVelocity(600);
            lm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // while (Math.abs(lm.getCurrentPosition() - targetPos)>=2){}
            
            motor_speed= 1;
            lm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            total_time = robot.time_for_dist(1.5 * robot.ground_tiles_width, Math.abs(motor_speed));
            timer.reset();
            while (opModeIsActive() && timer.seconds() < total_time) {
                lm.setPower(motor_speed);
                rm.setPower(motor_speed);
            }
            lm.setPower(0);
            rm.setPower(0);


        }
    }
}
