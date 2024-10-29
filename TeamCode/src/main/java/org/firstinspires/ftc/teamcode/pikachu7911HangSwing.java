package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class pikachu7911HangSwing extends LinearOpMode {

    static double outestOut = 0.7;
    static double innestIn = -0.35;

    static int highestHigh = 2300;
    static int lowestLow = 800;

    Servo leftHang;
    Servo rightHang;

    DcMotor leftSlide;
    DcMotor rightSlide;

    @Override
    public void runOpMode() throws InterruptedException {
        leftHang = hardwareMap.servo.get("leftHang");
        rightHang = hardwareMap.servo.get("rightHang");

        leftSlide = hardwareMap.dcMotor.get("leftSlide");
        rightSlide = hardwareMap.dcMotor.get("rightSlide");

        leftHang.setDirection(Servo.Direction.REVERSE);
        rightHang.setDirection(Servo.Direction.FORWARD);

        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlide.setDirection(DcMotorSimple.Direction.FORWARD);

        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad2.left_bumper) {
                leftHang.setPosition(outestOut);
                rightHang.setPosition(outestOut);
            }
            if (gamepad2.right_bumper) {
                leftHang.setPosition(innestIn);
                rightHang.setPosition(innestIn);
            }
            if (gamepad2.dpad_down) {
                leftHang.setPosition(leftHang.getPosition());
                rightHang.setPosition(rightHang.getPosition());
            }
            if (gamepad2.y) {
                leftSlide.setTargetPosition(highestHigh);
                rightSlide.setTargetPosition(highestHigh);
                leftSlide.setPower(1);
                rightSlide.setPower(1);
            }

            if (gamepad2.a) {
                leftSlide.setTargetPosition(lowestLow);
                rightSlide.setTargetPosition(lowestLow);
                leftSlide.setPower(-1);
                rightSlide.setPower(-1);
            }
            if (gamepad2.b) {
                leftSlide.setPower(0);
                rightSlide.setPower(0);
            }
        }
    }
}
