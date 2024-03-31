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
<<<<<<< HEAD
		harvestmotor = hardwareMap.get(DcMotor.class, "moissonneuse");
=======
		harvestmotor = hardwareMap.get(DcMotor.class, "moissoneuse");
>>>>>>> a8ae6d3 (hello)

		rm.setDirection(DcMotor.Direction.REVERSE);

		imu = hardwareMap.get(IMU.class, "imu");
		imu.initialize(
				new IMU.Parameters(
						new RevHubOrientationOnRobot(
								RevHubOrientationOnRobot.LogoFacingDirection.UP,
								RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
		imu.resetYaw();
		YawPitchRollAngles robotOrientation;
		FTC2024WeRobotControl robot = new FTC2024WeRobotControl(this);
		autonomous_mode = "b4d";

		telemetry.addData("wait for start", "");
		telemetry.update();

		waitForStart();
		telemetry.addData("started", "");
		telemetry.update();
		robotOrientation = imu.getRobotYawPitchRollAngles();

		if (opModeIsRunning()) {
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
					robot.forward(0.5);
					robot.rotate((-90));
					robot.forward(1.5);
					robot.harvest(-1);
					robot.backward(1);
					robot.harvest(0);
					break;
				case ("b2d"):
					robot.forward(0.5);
					robot.rotate((-90));
					robot.forward(2.5);
					robot.harvest(-1);
					robot.backward(1);
					robot.harvest(0);
					break;
				case ("r4d"):
					robot.forward(0.5);
					robot.rotate(90);
					robot.forward(1.5);
					robot.harvest(-1);
					robot.backward(1);
					robot.harvest(0);
					break;
				case ("r2d"):
					robot.forward(0.5);
					robot.rotate(90);
					robot.forward(2.5);
					robot.harvest(-1);
					robot.backward(1);
					robot.harvest(0);
					break;
				case ("b4n"):
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
					break;
				case ("b2n"):
					robot.forward(1.5);
					robot.rotate(90);
					robot.harvest();
					robot.forward(1);
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
					break;
				case ("r4n"):
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
					break;
				case ("r2n"):
					robot.forward(1.5);
					robot.rotate(-90);
					robot.harvest();
					robot.forward(1);
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
					break;
			}
		}
	}
}
