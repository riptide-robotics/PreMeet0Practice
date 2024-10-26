package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Slide Move")
public class SlideRotate extends LinearOpMode {
    Servo slideJoint;
    DcMotor slideMotor;
    final double rotateUp = 0.7;
    final double rotateDown = 0.3;
    final double rotateMiddle = 0.5;

    final int highestPos = 2350; //idk the value
    final int lowestPos = 100;

    @Override



    public void runOpMode() throws InterruptedException{
        slideJoint = hardwareMap.servo.get("pivotSlide");

        slideMotor = hardwareMap.dcMotor.get("slideMotor");

        slideMotor.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        while(opModeIsActive()){

            if (gamepad1.dpad_down)
            {
                slideJoint.setPosition(rotateDown);
            }

            if (gamepad1.dpad_up)
            {
                slideJoint.setPosition(rotateUp);
            }

            if (gamepad1.dpad_left)
            {
                slideJoint.setPosition(0);
            }

            if (gamepad1.dpad_left)
            {
                moveSlides(1, highestPos);
            }

            if (gamepad1.dpad_right)
            {
                moveSlides(-1, lowestPos);
            }
            if (gamepad1.left_bumper)
            {
                slideJoint.setPosition(rotateMiddle);
            }
        }
    }

    public void moveSlides(double power, int target)
    {
        slideMotor.setTargetPosition(target);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(power);
    }
}
