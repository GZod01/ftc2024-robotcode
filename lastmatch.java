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

@Autonomous

public class Ftc2024_autonome_last extends LinearOpMode {
    public enum AutoMode {
	B2D, B4D
    }

    public AutoMode autonomous_mode = B2D;
    public DcMotor lm;
    public DcMotor rm;
    public DcMotorEx lmelevator;
    public DcMotorEx rmelevator;
    public DcMotor harvestmotor;
    public IMU imu;
    public DcMotorEx rotation;

    @Override
    public void runOpMode() {

	boolean auto = false;
	lm = hardwareMap.get(DcMotor.class, "blm");
	rm = hardwareMap.get(DcMotor.class, "brm");
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
	autonomous_mode = AutoMode.B4D;

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
		    /*
		     * robot.harvest(0);
		     * robot.rotate((-90));
		     * robot.posBasse();
		     * robot.forward(3.5);
		     * robot.harvest(-1);
		     * robot.backward(0.5);
		     * robot.harvest(0);
		     * robot.forward(0.5);
		     */
		    break;
		case B2D:
		    robot.posBasse();
		    robot.harvest(1);
		    robot.forward(1);
		    robot.harvest(0);
		    /*
		     * robot.forward(1);
		     * robot.harvest(-1);
		     * robot.backward(0.5);
		     * robot.harvest(0);
		     * robot.forward(0.5);
		     */
		    break;
	    }
	}
    }
}