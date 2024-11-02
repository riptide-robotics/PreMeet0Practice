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
    Servo rSlideJoint;
    DcMotor rSlideMotor;

    // Left slides
    Servo lSlideJoint;
    DcMotor lSlideMotor;

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

        rSlideJoint = hardwareMap.servo.get("rSlideJoint");
        lSlideJoint = hardwareMap.servo.get("lSlideJoint");

        rSlideMotor = hardwareMap.dcMotor.get("rSlide");
        lSlideMotor = hardwareMap.dcMotor.get("lSlide");

        lSlideMotor.setDirection(DcMotor.Direction.REVERSE);


        rSlideJoint.setPosition(rotateDown);
        lSlideJoint.setPosition(rotateUp);

        telemetry.addData("Is it uploading?", "YEs");
        telemetry.update();

        rSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while(opModeIsActive()){

//           This was for trouble shooting could also be used to check encoder values
//            telemetry.addData("Slide Motor 1 Position: ", slideMotor1.getCurrentPosition());
//            telemetry.addData("Slide Motor 2 Position: ", slideMotor2.getCurrentPosition());
//            telemetry.update();

            double rSlideCurrentPos = rSlideJoint.getPosition();
            double lSlideCurrentPos = lSlideJoint.getPosition();
            // rotate up
            if (gamepad1.dpad_left)
            {
                rSlideJoint.setPosition(rotateUp);
                lSlideJoint.setPosition(rotateDown);
                telemetry.addData("Slides: ", "rotating up");
                telemetry.update();
            }

            // rotate down
            if (gamepad1.dpad_right)
            {
                rSlideJoint.setPosition(rotateDown);
                lSlideJoint.setPosition(rotateUp);
                telemetry.addData("Slides: ", "rotating down");
                telemetry.update();
            }

            // Both slides up
            if (gamepad1.dpad_up)
            {
                //moveSlides(1, highestPos);
                rSlideMotor.setPower(1);
                lSlideMotor.setPower(1);
                telemetry.addData("Slides: ", "moving up");
                telemetry.update();
            }

            // Both slides down
            if (gamepad1.dpad_down)
            {
                //moveSlides(-1, lowestPos);
                rSlideMotor.setPower(-1);
                lSlideMotor.setPower(-1);
                telemetry.addData("Slides: ", "moving down");
                telemetry.update();
            }

            // Both slides stopped
            if (gamepad1.right_bumper)
            {
                rSlideMotor.setPower(0);
                lSlideMotor.setPower(0);
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
                rSlideJoint.setPosition(rSlideCurrentPos - rServoPos); // IDK how to pause servo position help
                lSlideJoint.setPosition(lServoPos - lSlideCurrentPos); // IDK how to pause servo position help
                sleep(3000);
                telemetry.addData("Rotation Servo 1: ", rSlideCurrentPos);
                telemetry.addData("Rotation Servo 2: ", lSlideCurrentPos);
                telemetry.update();
            }

            // Reset Rotation
            if (gamepad1.left_bumper)
            {
                rSlideJoint.setPosition(rotateMiddle);
                lSlideJoint.setPosition(rotateMiddle);
                telemetry.addData("Rotation: ", "Reset");
                telemetry.update();
            }

            rServoPos = rSlideJoint.getPosition();
            lServoPos = lSlideJoint.getPosition();
        }
    }

    // set power to
    public void moveSlides(double power, int target)
    {
        //slideMotor1.setTargetPosition(target);
        //slideMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rSlideMotor.setPower(power);

        //slideMotor2.setTargetPosition(target);
        //slideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lSlideMotor.setPower(power);
    }
}
