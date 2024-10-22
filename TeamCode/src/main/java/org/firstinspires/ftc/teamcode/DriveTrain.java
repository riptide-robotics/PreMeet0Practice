package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class DriveTrain {


    private final Gamepad gamepad1;

    private final DcMotor frWheel;
    private final DcMotor flWheel;
    private final DcMotor brWheel;
    private final DcMotor blWheel;
    private final IMU imu;


    //We have to pass these in b/c only classes that extend OpMode can access these values normally.
    public DriveTrain(HardwareMap hardwareMap, Gamepad gamepad1) {

        this.gamepad1 = gamepad1;

        frWheel = hardwareMap.dcMotor.get("frWheel");
        flWheel = hardwareMap.dcMotor.get("flWheel");
        brWheel = hardwareMap.dcMotor.get("brWheel");
        blWheel = hardwareMap.dcMotor.get("blWheel");

        blWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        frWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        brWheel.setDirection(DcMotorSimple.Direction.FORWARD);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

        imu.resetYaw();
        imu.initialize(parameters);


    }

    public void robotCentricDrive() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1; // 1.1 is to account for hardware inconsistencies.
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frWheelPower = (y - x - rx) / denominator;
        double flWheelPower = (y + x + rx) / denominator;
        double brWheelPower = (y + x - rx) / denominator;
        double blWheelPower = (y - x + rx) / denominator;

        frWheel.setPower(frWheelPower);
        brWheel.setPower(brWheelPower);
        blWheel.setPower(blWheelPower);
        flWheel.setPower(flWheelPower);
    }

    public void fieldCentricDrive() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1; // 1.1 is to account for hardware inconsistencies.
        double rx = gamepad1.right_stick_x;

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); // heading of bot in radians

        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading); // Linear transformations yay
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1); // we like our drivers to have more control
        double frWheelPower = (rotY - rotX - rx) / denominator;
        double flWheelPower = (rotY + rotX + rx) / denominator;
        double brWheelPower = (rotY + rotX - rx) / denominator;
        double blWheelPower = (rotY - rotX + rx) / denominator;

        frWheel.setPower(frWheelPower);
        brWheel.setPower(brWheelPower);
        blWheel.setPower(blWheelPower);
        flWheel.setPower(flWheelPower);
    }


}
