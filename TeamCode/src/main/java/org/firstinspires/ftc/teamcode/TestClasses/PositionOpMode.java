package org.firstinspires.ftc.teamcode.TestClasses;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Autonomous.EditablePose2D;
import org.firstinspires.ftc.teamcode.Autonomous.OdometryLocalizer;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "Odometry Test")
public class PositionOpMode extends LinearOpMode {

    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * * * * * * * * * * * * * * *
         * INITIALIZATION
         * * * * * * * * * * * * * * *
         */
        robot = new Robot(hardwareMap);

        telemetry.addData("Robot status", "succesfully initiated");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        /*
         * * * * * * * * * * * * * * *
         * Start button clicked
         * * * * * * * * * * * * * * *
         */
        telemetry.clear();
        robot.startOdometry();

        /*
         * * * * * * * * * * * * * * *
         * LOOP
         * * * * * * * * * * * * * * *
         */

        while(opModeIsActive()) {

            fieldCentricDrive();
            EditablePose2D currPos = robot.getCurrPos();

            telemetry.addData("X Position", currPos.getX(DistanceUnit.INCH));
            telemetry.addData("Y Position", currPos.getY(DistanceUnit.INCH));
            telemetry.addData("Orientation (Degrees)", Math.toDegrees(currPos.getH()));

            telemetry.addLine("\n Raw Values \n")
                            .addData("leftEncoder", robot.getRobotPos().getLeftEncoder())
                            .addData("rightEncoder", robot.getRobotPos().getRightEncoder())
                            .addData("perpendicularEncoder", robot.getRobotPos().getPerpendicularEncoder());

            telemetry.addLine("\n IMU measured heading")
                            .addData("Orientation (Degrees)", robot.getRobotHeading(AngleUnit.DEGREES));

            telemetry.update();
        }

    }

    public void fieldCentricDrive() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1; // 1.1 is to account for hardware inconsistencies.
        double rx = gamepad1.right_stick_x;

        double heading = robot.getRobotHeading(AngleUnit.RADIANS); // heading of bot in radians

        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading); // Linear transformations yay
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1); // we like our drivers to have more control
        double frWheelPower = (rotY - rotX - rx) / denominator;
        double flWheelPower = (rotY + rotX + rx) / denominator;
        double brWheelPower = (rotY + rotX - rx) / denominator;
        double blWheelPower = (rotY - rotX + rx) / denominator;

        robot.setWheelPowers(flWheelPower, frWheelPower, brWheelPower, blWheelPower);
    }
}
