package org.firstinspires.ftc.teamcode;

//import FTC2024WeRobotControl; //a tester car pas sur que ça fonctionne

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

public class Ftc2024_autonome_api extends LinearOpMode {
    public String autonomous_mode;
    public DcMotor lm;
    public DcMotor rm;
    public DcMotor harvestmotor;
    public IMU imu;
    
    
    
    
    @Override
    public void runOpMode() {
    lm = hardwareMap.get(DcMotor.class, "blm");
    rm = hardwareMap.get(DcMotor.class, "brm");
    harvestmotor = hardwareMap.get(DcMotor.class, "flm");

    rm.setDirection(DcMotor.Direction.REVERSE);

    imu = hardwareMap.get(IMU.class, "imu");
    imu.initialize(
        new IMU.Parameters(
            new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            )
            )
        );
    imu.resetYaw();
    YawPitchRollAngles robotOrientation;
    FTC2024WeRobotControl robot = new FTC2024WeRobotControl(this);
    autonomous_mode = "b4d";
    
    telemetry.addData("wait for start","");
    telemetry.update();

    waitForStart();
    telemetry.addData("started","");
    telemetry.update();
    robotOrientation = imu.getRobotYawPitchRollAngles();
    
    if(opModeIsActive()){
        /*
         * autonomous_mode differents possibles values respect the next scheme:
         * team_color_shortcode + start_line_index + direct_or_no
         *
         * team_color_shortcode = "b" for blue & "r" for red
         * start_line_index = 4 or 2 see competition manual appendix B Tile location plan
         * direct_or_no = "d" to direct go to pixel deliver zone or "n" to harvest pixels before to go in deliver zone
         *
         * default is "b4d"
         */
        switch (autonomous_mode){
        case ("b4d"):
            telemetry.addData("cas b4d lancé","");
            telemetry.update();
            robot.forward(0.5);
            robot.rotate(-90.0);
            robot.forward(1.5);
            robot.harvest(-1);
            robot.backward(1);
            robot.harvest(0);
            telemetry.addData("fin de b4d","");
            telemetry.update();
            break;
        case ("b2d"):
            telemetry.addData("cas b2d lancé","");
            telemetry.update();
            robot.forward(0.5);
            robot.rotate(-90.0);
            robot.forward(2.5);
            robot.harvest(-1);
            robot.backward(1);
            robot.harvest(0);
            telemetry.addData("fin de b2d","");
            telemetry.update();
            break;
        case ("r4d"):
            telemetry.addData("cas r4d lancé","");
            telemetry.update();
            robot.forward(0.5);
            robot.rotate(90.0);
            robot.forward(1.5);
            robot.harvest(-1);
            robot.backward(1);
            robot.harvest(0);
            telemetry.addData("fin de r4d","");
            telemetry.update();
            break;
        case ("r2d"):
            telemetry.addData("cas r2d lancé","");
            telemetry.update();
            robot.forward(0.5);
            robot.rotate(90.0);
            robot.forward(2.5);
            robot.harvest(-1);
            robot.backward(1);
            robot.harvest(0);
            telemetry.addData("fin de r2d","");
            telemetry.update();
            break;

        case ("b4n"):

            break;
        case ("b2n"):
            
            break;
        case ("r4n"): 
            
            break;
        case ("r2n"):

            break;
        default:
            break;
        }
    }
    }
}
