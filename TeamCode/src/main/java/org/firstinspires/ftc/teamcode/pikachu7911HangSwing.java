package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class pikachu7911HangSwing extends LinearOpMode {

    Servo leftHang;
    Servo rightHang;

    @Override
    public void runOpMode() throws InterruptedException {
        leftHang = hardwareMap.servo.get("leftHang");
        rightHang = hardwareMap.servo.get("rightHang");

        leftHang.setDirection(Servo.Direction.REVERSE);
        rightHang.setDirection(Servo.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad2.left_bumper) {
                leftHang.setPosition(0.7);
                rightHang.setPosition(0.7);
            }
            if (gamepad2.right_bumper) {
                leftHang.setPosition(-0.35);
                rightHang.setPosition(-0.35);
            }
            if (gamepad2.dpad_down) {
                leftHang.setPosition(leftHang.getPosition());
                rightHang.setPosition(rightHang.getPosition());
            }s
        }
    }
}
