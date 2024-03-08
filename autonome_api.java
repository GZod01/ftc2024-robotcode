package org.firstinspires.ftc.teamcode;

import fr.werobot.ftc2024.robotcontrol.FTC2024WeRobotControl;

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

public class ftc2024_autonome_api extends LinearOpMode {
    public DcMotor lm;
    public DcMotor rm;
    public DcMotor harvestmotor;
    public IMU imu;
    public YawPitchRollAngle robotOrientation;
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
	FTC2024WeRobotControl robot = FTC2024WeRobotControl(this);

	waitForStart();
	robotOrientation = imu.getYawPitchRollAngles();

	if(opModeIsRunning()){
	    robot.forward(0.5);
	    robot.rotate(-90.0);
	    robot.forward(1.5);
	    robot.harvest(-1);
	    robot.backward(1);
	    robot.harvest(0);
	}
    }
}
