package org.firstinspires.ftc.teamcode;

// importing stuff
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "MechanimWheels")

public class MechanimWheels extends LinearOpMode{

    // importing dc motors
    DcMotor flWheel;
    DcMotor frWheel;
    DcMotor blWheel;
    DcMotor brWheel;

    double x ;
    double y;
    double rx;

    @Override

    public void runOpMode() throws InterruptedException
    {
        // reference our motors so program knows what we calling
        flWheel = hardwareMap.dcMotor.get("flWheel");
        frWheel = hardwareMap.dcMotor.get("frWheel");
        blWheel = hardwareMap.dcMotor.get("blWheel");
        brWheel = hardwareMap.dcMotor.get("brWheel");

        waitForStart();

        // Reverse direction because gamepad standard is reversed (only for left side)
        flWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        blWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        while (opModeIsActive())
        {
            // referencing gamepad stick to varialbe (x multiplied by 1.1 for margine of error
            y = -gamepad1.left_stick_y;
            x = -gamepad1.left_stick_x * 1.1;
            rx = gamepad1.right_stick_x;

            // powers of each motor
            double flPower = y-x+rx;
            double frPower = y+x-rx;
            double blPower = y+x+rx;
            double brPower = y-x-rx;

            double scale = Math.max(Math.abs(y)+Math.abs(x)+Math.abs(rx), 1);


            // Set power to each motor
            flWheel.setPower(flPower);
            frWheel.setPower(frPower);
            blWheel.setPower(blPower);
            brWheel.setPower(brPower);
        }



    }


}
