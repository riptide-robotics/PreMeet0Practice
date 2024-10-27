package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class pikachu7911FieldCentricDrive {

    private Gamepad gamepad1;

    private DcMotor flWheel, frWheel, blWheel, brWheel;

    private IMU imu;

    private ElapsedTime time = new ElapsedTime();

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

        flWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
    //Pose2D **********************************************************************
    private double xPos = 0; // the X value in the math
    private double yPos = 0; // the Y value in the math
    private double rads = 0;
    private double oldOdo1 = 0;
    private double oldOdo2 = 0;
    private double oldOdo3 = 0;

    public void threeOdometryLocalization() {
        final double xWidth = 123120; // will be set later
        final double perpCenter = 61560; // will be set later
        final double odoWheelRadius = 4.8;
        final int ticksPerRev = 2000;

        final double distancePerTick = 2 * Math.PI * odoWheelRadius / ticksPerRev;

        double odo1 = frWheel.getCurrentPosition();
        double odo2 = flWheel.getCurrentPosition();
        double odo3 = brWheel.getCurrentPosition();

        double dodo1 = odo1 - oldOdo1;
        double dodo2 = odo2 - oldOdo2;
        double dodo3 = odo3 - oldOdo3;

        double dx = distancePerTick * (dodo1 + dodo2) / 2;
        double dtheta = distancePerTick * (dodo1 - dodo2) / xWidth;
        double dy = distancePerTick * dodo3 - perpCenter * dtheta;
        dx = dx * Math.cos(rads) - dx * Math.sin(rads);
        dy = dy * Math.cos(rads) + dy * Math.cos(rads);

        oldOdo1 = odo1;
        oldOdo2 = odo2;
        oldOdo3 = odo3;

        xPos += dx;
        yPos += dy;
        rads += dtheta;
    }

    private static double kPy = 0;
    private static double kIy = 0;
    private static double kDy = 0;
    private static double kPx = 0;
    private static double kIx = 0;
    private static double kDx = 0;

    public double integralx = 0;
    public double integraly = 0;
    public double previousErrorx = 0;
    public double previousErrory = 0; // Adjust as needed

    ElapsedTime PIDtime = new ElapsedTime();

    public void drivePIDx(int goalPos) {
        double currPos = xPos;
        double err = goalPos - currPos;

        double elapsedTime = PIDtime.seconds();

        integralx += err * elapsedTime;

        double integralLimit = 100000;
        if (Math.abs(integralx) > integralLimit) {
            integralx = Math.signum(integralx) * integralLimit;
        }

        double derivative = (err - previousErrorx) / elapsedTime;

        double output = (kPx * err) + (kIx * integralx) + (kDx * derivative);

        flWheel.setPower(output);
        frWheel.setPower(output);
        blWheel.setPower(output);
        brWheel.setPower(output);

        previousErrorx = err;
        PIDtime.reset();
    }
    public void drivePIDy(int goalPos) {
        double currPos = yPos;
        double err = goalPos - currPos;

        double elapsedTime = PIDtime.seconds();

        integraly += err * elapsedTime;

        double integralLimit = 100000;
        if (Math.abs(integraly) > integralLimit) {
            integraly = Math.signum(integraly) * integralLimit;
        }

        double derivative = (err - previousErrory) / elapsedTime;

        double output = (kPy * err) + (kIy * integraly) + (kDy * derivative);

        flWheel.setPower(output);
        frWheel.setPower(-output);
        blWheel.setPower(-output);
        brWheel.setPower(output);

        previousErrory = err;
        PIDtime.reset();
    }

    public void resetIntegralx() {
        integralx = 0;
    }
    public void resetIntegraly() {
        integraly = 0;
    }
}
