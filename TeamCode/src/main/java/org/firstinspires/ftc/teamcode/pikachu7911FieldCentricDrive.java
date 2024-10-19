package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class pikachu7911FieldCentricDrive {

    private Gamepad gamepad1;

    private DcMotor flWheel, frWheel, blWheel, brWheel;

    private IMU imu;


    public pikachu7911FieldCentricDrive (HardwareMap hardwareMap, Gamepad gamepad1)
    {
        this.gamepad1 = gamepad1;

        flWheel = hardwareMap.dcMotor.get("flWheel");
        frWheel = hardwareMap.dcMotor.get("frWheel");
        blWheel = hardwareMap.dcMotor.get("blWheel");
        brWheel = hardwareMap.dcMotor.get("brWheel");

        flWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        blWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        frWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        brWheel.setDirection(DcMotorSimple.Direction.FORWARD);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        ));

        imu.resetYaw();
        imu.initialize(parameters);
    }

    public void fieldCentricDrive() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frWheelPower = (rotY - rotX - rx) / denominator;
        double flWheelPower = (rotY + rotX + rx) / denominator;
        double brWheelPower = (rotY + rotX - rx) / denominator;
        double blWheelPower = (rotY - rotX + rx) / denominator;

        frWheel.setPower(frWheelPower);
        flWheel.setPower(flWheelPower);
        brWheel.setPower(brWheelPower);
        blWheel.setPower(blWheelPower);
    }

    public void threeOdometryLocalization() {
        final double xWidth;
        final double perpCenter;
        final double odoWheelRadius;
    }
}
