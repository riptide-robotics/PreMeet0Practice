package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Field Centric Drive")
public class FieldCentricDrive extends LinearOpMode {


    // Stating dc motors
    DcMotor flWheel;
    DcMotor frWheel;
    DcMotor blWheel;
    DcMotor brWheel;

    double x;
    double y;
    double rx;


    private final double R = 4.8; //
    private final int V = 2000; //
    private final double C = 1/V; // Circumfrence
    private final double L = 10; //unknown I PUT A RANDOM NUMBER (L is the distance between the 2 parallel odomitors


    private IMU imu;



    @Override
    public void runOpMode() throws InterruptedException {

        // reference our motors so program knows what we calling
        flWheel = hardwareMap.dcMotor.get("flWheel");
        frWheel = hardwareMap.dcMotor.get("frWheel");
        blWheel = hardwareMap.dcMotor.get("blWheel");
        brWheel = hardwareMap.dcMotor.get("brWheel");

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters paremeters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

        imu.resetYaw();

        imu.initialize(paremeters);



        int odomitorParallel1Init = flWheel.getCurrentPosition() ;
        int odomitorParallel2Init = frWheel.getCurrentPosition();
        int odomitorPerpendicularInit = blWheel.getCurrentPosition();

        waitForStart();

        // Reverse direction because gamepad standard is reversed (only for left side)
        flWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        blWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        while (opModeIsActive()) {
            // referencing gamepad stick to varialbe (x multiplied by 1.1 for margine of error
            y = -gamepad1.left_stick_y;
            x = -gamepad1.left_stick_x * 1.1;
            rx = gamepad1.right_stick_x;


            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotY = x * Math.sin(-heading) + y * Math.sin(-heading);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);



            double scale = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            // powers of each motor
            double flPower = (rotY - rotX + rx) / denominator;
            double frPower = (rotY + rotX - rx) / denominator;
            double blPower = (rotY + rotX + rx) / denominator;
            double brPower = (rotY - rotX - rx) / denominator;


            // Set power to each motor
            flWheel.setPower(flPower);
            frWheel.setPower(frPower);
            blWheel.setPower(blPower);
            brWheel.setPower(brPower);

            int odomitorParallel1Change = flWheel.getCurrentPosition() - odomitorParallel1Init; // Front left odomitor
            int odomitorParallel2Change = frWheel.getCurrentPosition() - odomitorParallel2Init; // Front right odomitor
            int odomitorPerpendicularChange = blWheel.getCurrentPosition() - odomitorPerpendicularInit; // Perpendicular odomitor

            double deltaY = odomitorPerpendicularChange - (odomitorParallel2Change - odomitorParallel1Change) / 2;
            double deltaX = C*(odomitorParallel1Change + odomitorParallel2Change);
            double deltaTheta = C*(odomitorParallel2Change - odomitorParallel1Change) / L;


            odomitorParallel1Init = flWheel.getCurrentPosition();
            odomitorParallel2Init = frWheel.getCurrentPosition();
            odomitorPerpendicularInit = blWheel.getCurrentPosition();



        }
    }
}
