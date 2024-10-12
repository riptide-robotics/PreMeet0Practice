package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "pikachu", group = "driveTrain")
public class pikachu7911MecanumDrive extends LinearOpMode {

    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize on hardwareMap
        FL = hardwareMap.dcMotor.get("flWheel");
        FR = hardwareMap.dcMotor.get("frWheel");
        BL = hardwareMap.dcMotor.get("blWheel");
        BR = hardwareMap.dcMotor.get("brWheel");

        // Practice
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);

        // It's only been initialized so far
        waitForStart();

        // while Driving
        while (opModeIsActive()) {
            // Set the sticks
            double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Fix incorrect strafing
            double rx = gamepad1.right_stick_x;

            // -1 <= Scale <= 1
            double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            // Set the motors
            FL.setPower((y + x + rx) / denom);
            FR.setPower((y - x - rx) / denom);
            BL.setPower((y - x + rx) / denom);
            BR.setPower((y + x - rx) / denom);
        }
    }
}
