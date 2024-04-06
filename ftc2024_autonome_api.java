package org.firstinspires.ftc.teamcode;

//import FTC2024WeRobotControl; //a tester car pas sur que ça fonctionne

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
import com.qualcomm.robotcode.hardware.HardwareMap;

@Autonomous

public class Ftc2024_autonome_api{
    public enum AutoMode {
        B2D, B4D, B2N, B4N, R2D, R4D, R2N, R4N
    }
    public HardwareMap hardwareMap;
    public AutoMode autonomous_mode;
    public DcMotorEx lm;
    public DcMotorEx rm;
    public DcMotorEx lmelevator;
    public DcMotorEx rmelevator;
    public DcMotor harvestmotor;
    public IMU imu;
    public DcMotorEx rotation;

    public void runOpMode() {

        boolean auto = false;
        lm = hardwareMap.get(DcMotorEx.class, "blm");
        rm = hardwareMap.get(DcMotorEx.class, "brm");
        harvestmotor = hardwareMap.get(DcMotor.class, "moissonneuse");
        rotation = hardwareMap.get(DcMotorEx.class, "elvRot");
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

        while (opModeInInit()){
            imu.resetYaw();
            telemetry.addData("wait", "for start");
            telemetry.addData("Yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }
        
        while (opModeIsActive()) {
            if (gamepad1.a && !auto) {
                auto = true;
                break;
            }
        }
        if (opModeIsActive()) {
            /*
             * autonomous_mode differents possibles values respect the next scheme:
             * team_color_shortcode + start_line_index + direct_or_no
             *
             * team_color_shortcode = "b" for blue & "r" for red
             * start_line_index = 4 or 2 see competition manual appendix B Tile location
             * plan
             * direct_or_no = "d" to direct go to pixel deliver zone or "n" to harvest
             * pixels before to go in deliver zone
             *
             * default is "b4d"
             */

            switch (autonomous_mode) {
                default:
                    robot.boxElv();
                    robot.harvest(1);
                    robot.forward(2);
                    /*robot.harvest(0);
                    robot.rotate((-90));
                    robot.posBasse();
                    while (opModeIsActive() && rotation.getCurrentPosition() < rotation.getTargetPosition()) {
                        telemetry.addData("pos", rotation.getCurrentPosition());
                        telemetry.update();
                    }
                    robot.forward(3.5);
                    robot.harvest(-1);
                    robot.backward(0.5);
                    robot.harvest(0);
                    robot.forward(0.5);*/
                    break;
                case B2D:
                    imu.resetYaw();
                    
                    robot.posBasse();
                    while (opModeIsActive() && rotation.getCurrentPosition() < rotation.getTargetPosition()) {
                        telemetry.addData("pos", rotation.getCurrentPosition());
                        telemetry.addData("Yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                        telemetry.update();
                    }
                    robot.harvest(1);
                    robot.forward(1);
                    robot.harvest(0);
                   /* robot.forward(1);
                    robot.harvest(-1);
                    robot.backward(0.5);
                    robot.harvest(0);
                    robot.forward(0.5);*/
                    break;
                case R4D:
                    robot.boxElv();
                    robot.harvest(1);
                    robot.forward(2);
                    robot.harvest(0);
                    robot.rotate(90);
                    robot.posBasse();
                    while (opModeIsActive() && rotation.getCurrentPosition() < rotation.getTargetPosition()) {
                        telemetry.addData("pos", rotation.getCurrentPosition());
                        telemetry.update();
                    }
                    robot.forward(3.5);
                    robot.harvest(-1);
                    robot.backward(0.5);
                    robot.harvest(0);
                    robot.forward(1);
                    break;
                case R2D:
                    robot.boxElv();
                    //rebuild
                    //robot.harvest(1);
                    robot.forward(1.5);
                    //robot.harvest(0);
                    robot.rotate(90);
                    robot.posBasse();
                    while (opModeIsActive() && rotation.getCurrentPosition() < rotation.getTargetPosition()) {
                        telemetry.addData("pos", rotation.getCurrentPosition());
                        telemetry.update();
                    }
                    robot.forward(1.5);
                    //robot.harvest(-1);
                    //robot.backward(0.5);
                    //robot.harvest(0);
                    //robot.forward(0.5);
                    break;
                case B4N:
                    robot.boxElv();
                    robot.forward(1.5);
                    robot.rotate(90);
                    robot.harvest();
                    robot.forward(3);
                    robot.harvest(0);
                    robot.rotate(180);
                    robot.forward(1);
                    robot.rotate(-90);
                    robot.forward(1);
                    robot.rotate(90);
                    robot.forward(2.5);
                    robot.harvest(-1);
                    robot.backward(1);
                    robot.harvest(0);
                    robot.forward(1);
                    break;
                case B2N:
                    robot.boxElv();
                    robot.forward(1.5);
                    robot.rotate(90);
                    robot.harvest();
                    robot.forward(1);
                    robot.harvest(0);
                    robot.forward(1);
                    robot.rotate(180);
                    robot.forward(1);
                    robot.rotate(-90);
                    robot.forward(1);
                    robot.rotate(90);
                    robot.forward(2.5);
                    robot.harvest(-1);
                    robot.backward(1);
                    robot.harvest(0);
                    robot.forward(1);
                    break;
                case R4N:
                    robot.boxElv();
                    robot.forward(1.5);
                    robot.rotate(-90);
                    robot.harvest();
                    robot.forward(3);
                    robot.harvest(0);
                    robot.rotate(180);
                    robot.forward(1);
                    robot.rotate(90);
                    robot.forward(1);
                    robot.rotate(-90);
                    robot.forward(2.5);
                    robot.harvest(-1);
                    robot.backward(1);
                    robot.harvest(0);
                    robot.forward(1);
                    break;
                case R2N:
                    robot.boxElv();
                    robot.forward(1.5);
                    robot.rotate(-90);
                    robot.harvest();
                    robot.forward(1);
                    robot.harvest(0);
                    robot.forward(1);
                    robot.rotate(180);
                    robot.forward(1);
                    robot.rotate(90);
                    robot.forward(1);
                    robot.rotate(-90);
                    robot.forward(2.5);
                    robot.harvest(-1);
                    robot.backward(1);
                    robot.harvest(0);
                    robot.forward(1);
                    break;
            }
        }
    }
}
