package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Intake Skeleton")
public class WilliamIntakeSkeleton extends LinearOpMode {
    CRServo crServoLeft;
    CRServo crServoRight;

    Servo extendServoLeft;
    Servo extendServoRight;
    Servo intakePitchServo;
    Servo angleIntakeServoLeft;
    Servo angleIntakeServoRight;

    private final double minExtend = 0 /*Unknown*/;
    private final double maxExtend = 0 /*Unknown*/;
    private final double minPitch = 0 /*Unknown*/;
    private final double maxPitch = 0 /*Unknown*/;
    private final double minAngle = 0 /*Unknown*/;
    private final double maxAngle = 0 /*Unknown*/;



    public void runOpMode() {
        crServoLeft = hardwareMap.crservo.get("crservo1"); // configure this
        crServoRight = hardwareMap.crservo.get("crservo2");

        extendServoLeft = hardwareMap.servo.get("extendservoleft");
        extendServoRight = hardwareMap.servo.get("extendservoright");
        intakePitchServo = hardwareMap.servo.get("pitchServo");
        angleIntakeServoLeft = hardwareMap.servo.get("angleServol");
        angleIntakeServoRight = hardwareMap.servo.get("angleServor");

        waitForStart();

        while(opModeIsActive()) {
            if(gamepad1.a) {
                crServoLeft.setPower(0);
                crServoRight.setPower(0);
            } else if(gamepad1.left_bumper) {
                crServoLeft.setPower(1);
                crServoRight.setPower(-1);
            } else if(gamepad1.right_bumper) {
                crServoLeft.setPower(1);
                crServoRight.setPower(-1);
            }

            if(gamepad1.dpad_down) {
                extendServoLeft.setPosition(0);
                extendServoRight.setPosition(0);
            }

            if(gamepad1.dpad_up) {
                extendServoLeft.setPosition(maxExtend);
                extendServoRight.setPosition(maxExtend);
            }

            if(gamepad1.dpad_left) {
                angleIntakeServoLeft.setPosition(minAngle);
                angleIntakeServoRight.setPosition(minAngle);
            }

            if(gamepad1.dpad_right) {
                angleIntakeServoLeft.setPosition(maxAngle);
                angleIntakeServoRight.setPosition(maxAngle);
            }

            if(gamepad1.left_trigger >= 0.1) {
                intakePitchServo.setPosition(minPitch);
            }

            if(gamepad1.right_trigger >= 0.1) {
                intakePitchServo.setPosition(maxPitch);
            }
        }
    }
}
