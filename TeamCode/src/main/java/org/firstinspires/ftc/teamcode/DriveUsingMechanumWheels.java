package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "ExampleMechanumDrive", group = "Meet0")
public class DriveUsingMechanumWheels extends LinearOpMode {

    // Allocate Memory
    DcMotor frWheel;
    DcMotor flWheel;
    DcMotor brWheel;
    DcMotor blWheel;

    @Override
    public void runOpMode() throws InterruptedException {

        // When init button is hit

        frWheel = hardwareMap.dcMotor.get("frWheel");
        flWheel = hardwareMap.dcMotor.get("flWheel");
        brWheel = hardwareMap.dcMotor.get("brWheel");
        blWheel = hardwareMap.dcMotor.get("blWheel");

        frWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        // All code here is after the start button is hit.
        // Main loop; until stop button is hit
        while(opModeIsActive()){

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1; // 1.1 is to account for hardware inconsistencies.
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frWheelPower = (y - x - rx)/denominator;
            double flWheelPower = (y + x + rx)/denominator;
            double brWheelPower = (y + x - rx)/denominator;
            double blWheelPower = (y - x + rx)/denominator;

            frWheel.setPower(frWheelPower);
            brWheel.setPower(brWheelPower);
            blWheel.setPower(blWheelPower);
            flWheel.setPower(flWheelPower);

        }
    }
}
