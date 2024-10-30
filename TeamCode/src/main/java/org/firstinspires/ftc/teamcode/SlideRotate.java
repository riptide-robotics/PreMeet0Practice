package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@Config
@TeleOp(name = "Slide Move")
public class SlideRotate extends LinearOpMode {
    // Right slides
    Servo slideJoint1;
    DcMotor slideMotor1;

    // Left slides
    Servo slideJoint2;
    DcMotor slideMotor2;

    public static double rotateUp = 0.7;
    public static double rotateDown = 0.3;
    public static double rotateMiddle = 0.5;

    public static int highestPos = 2500;
    public static int lowestPos = 100;

    private double rServoPos = 0;
    private double lServoPos = 0;

    public static double hangPos = 0.4;

    @Override

    public void runOpMode() throws InterruptedException{
        slideJoint1 = hardwareMap.servo.get("rSlideJoint");
        slideJoint2 = hardwareMap.servo.get("lSlideJoint");
        slideMotor1 = hardwareMap.dcMotor.get("rSlide");
        slideMotor2 = hardwareMap.dcMotor.get("lSlide");

        slideMotor2.setDirection(DcMotor.Direction.REVERSE);


        slideJoint1.setPosition(rotateDown);
        slideJoint2.setPosition(rotateUp);

        telemetry.addData("Is it uploading?", "YEs");
        telemetry.update();

        slideMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while(opModeIsActive()){

//           This was for trouble shooting could also be used to check encoder values
//            telemetry.addData("Slide Motor 1 Position: ", slideMotor1.getCurrentPosition());
//            telemetry.addData("Slide Motor 2 Position: ", slideMotor2.getCurrentPosition());
//            telemetry.update();


            // rotate up
            if (gamepad1.dpad_left)
            {
                slideJoint1.setPosition(rotateUp);
                slideJoint2.setPosition(rotateDown);
                telemetry.addData("Slides: ", "rotating up");
                telemetry.update();
            }

            // rotate down
            if (gamepad1.dpad_right)
            {
                slideJoint1.setPosition(rotateDown);
                slideJoint2.setPosition(rotateUp);
                telemetry.addData("Slides: ", "rotating down");
                telemetry.update();
            }

            // Both slides up
            if (gamepad1.dpad_up)
            {
                //moveSlides(1, highestPos);
                slideMotor1.setPower(1);
                slideMotor2.setPower(1);
                telemetry.addData("Slides: ", "moving up");
                telemetry.update();
            }

            // Both slides down
            if (gamepad1.dpad_down)
            {
                //moveSlides(-1, lowestPos);
                slideMotor1.setPower(-1);
                slideMotor2.setPower(-1);
                telemetry.addData("Slides: ", "moving down");
                telemetry.update();
            }

            // Both slides stopped
            if (gamepad1.right_bumper)
            {
                slideMotor1.setPower(0);
                slideMotor2.setPower(0);
                telemetry.addData("Slides: ", "stopped");
                telemetry.update();
            }

            // INDIVIDUALLY MOVE RIGHT/LEFT SLIDES UP
//-------------------------------------------------------------------------------------------------------------------------------
//            // Right slide moving up
//            if (gamepad1.b)
//            {
//                slideMotor1.setPower(1);
//                slideMotor2.setPower(0);
//                telemetry.addData("Right Slide: ", "moving up");
//                telemetry.update();
//            }
//
//            // Left Slide moving up
//            if (gamepad1.x)
//            {
//                slideMotor1.setPower(0);
//                slideMotor2.setPower(1);
//                telemetry.addData("Left Slide: ", "moving up");
//                telemetry.update();
//            }
//
//            // Right Slide moving down
//            if (gamepad1.a)
//            {
//                slideMotor1.setPower(-1);
//                slideMotor2.setPower(0);
//                telemetry.addData("Right Slide: ", "moving down");
//                telemetry.update();
//            }
//
//            // Left slide moving down
//            if (gamepad1.y)
//            {
//                slideMotor1.setPower(0);
//                slideMotor2.setPower(-1);
//                telemetry.addData("Left Slide: ", "moving down");
//                telemetry.update();
//            }
//------------------------------------------------------------------------------------------------------------------------------------
            // Rotation stopped
            if (gamepad1.x)
            {
                //slideJoint1.setPosition(hangPos);
                //slideJoint2.setPosition(1 - hangPos);

                slideJoint1.setPosition(slideJoint1.getPosition());
                slideJoint2.setPosition(slideJoint2.getPosition());

                telemetry.addData("Rotation Servo 1: ", hangPos);
                telemetry.addData("Rotation Servo 2: ", lServoPos);
                telemetry.update();
            }

            // Reset Rotation
            if (gamepad1.left_bumper)
            {
                slideJoint1.setPosition(rotateMiddle);
                slideJoint2.setPosition(rotateMiddle);
                telemetry.addData("Rotation: ", "Reset");
                telemetry.update();
            }

            rServoPos = slideJoint1.getPosition();
            lServoPos = slideJoint2.getPosition();
        }
    }


    // set power to
    public void moveSlides(double power, int target)
    {
        //slideMotor1.setTargetPosition(target);
        //slideMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor1.setPower(power);

        //slideMotor2.setTargetPosition(target);
        //slideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor2.setPower(power);
    }
}
