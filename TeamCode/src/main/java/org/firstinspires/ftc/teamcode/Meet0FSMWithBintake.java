package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "0 Meet", group = "meet 0")
public class Meet0FSMWithBintake extends LinearOpMode {

    //Initializations
    DcMotor frWheel;
    DcMotor flWheel;
    DcMotor brWheel;
    DcMotor blWheel;

    DcMotor lLiftSlide;
    DcMotor rLiftSlide;

    Servo specimenClawGrab;
    Servo specimenClawPitch;

    Servo bintakeJoint;
    CRServo bintake;

    final double HANSEN_CLAW_MIN_PITCH = 0.8;
    final double HANSEN_CLAW_MEDIUM_PITCH = 0.61;
    final double HANSEN_CLAW_MAX_PITCH = 0.5;
    final double HANSEN_CLAW_MAX_GRAB = 1;
    final double HANSEN_CLAW_MIN_GRAB = 0.75;

    final double BINTAKE_UP = 1;
    final double BINTAKE_DOWN = 0.45;

    public static int HIGHEST_SLIDE_HEIGHT = 2350;
    public static int LOWEST_SLIDE_HEIGHT = 200;
    final int bintakeTolorance = 1500;

    //FSM statesu7
    public enum states{
        START,
        GRAB,
        DROP,
        HANG,
        BINTAKE,
        DEPLOY
    }
    @Override
    public void runOpMode() throws InterruptedException {

        //Drivetrain wheels
        frWheel = hardwareMap.dcMotor.get("frWheel");
        flWheel = hardwareMap.dcMotor.get("flWheel");
        brWheel = hardwareMap.dcMotor.get("brWheel");
        blWheel = hardwareMap.dcMotor.get("blWheel");

        blWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        frWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        brWheel.setDirection(DcMotorSimple.Direction.FORWARD);

        //Hansen Claw
        specimenClawGrab = hardwareMap.servo.get("specimenGrab");
        specimenClawPitch = hardwareMap.servo.get("specimenPitch");

        //Lift Slides
        lLiftSlide = hardwareMap.dcMotor.get("lSlide");
        rLiftSlide = hardwareMap.dcMotor.get("rSlide");

        lLiftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rLiftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lLiftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rLiftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lLiftSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        //Maksim Bintake
        bintakeJoint = hardwareMap.servo.get("bintakeJoint");
        bintake = hardwareMap.crservo.get("bintake");

        // init positions
        specimenClawPitch.setPosition(0.5);
        specimenClawGrab.setPosition(1);

        states currState = states.START;

        waitForStart();

        while(opModeIsActive())
        {

            switch (currState)
            {
                case START:

                    runSlides(0.8, LOWEST_SLIDE_HEIGHT);
                    specimenClawGrab.setPosition(HANSEN_CLAW_MAX_GRAB);
                    specimenClawPitch.setPosition(HANSEN_CLAW_MEDIUM_PITCH);

                    if(gamepad2.x)
                    {
                        specimenClawGrab.setPosition(HANSEN_CLAW_MIN_GRAB);
                        //runSlides(0.8, LOWEST_SLIDE_HEIGHT + 400);
                        currState = states.GRAB;
                    }

                    if(gamepad2.y)
                    {
                        currState = states.BINTAKE;
                    }


                    if(gamepad1.start)
                    {
                        currState = states.HANG;
                    }

                    break;
                case GRAB:

                    if(gamepad2.back)
                    {
                        runSlides(0.8, LOWEST_SLIDE_HEIGHT + 400);
                    }

                    if (gamepad2.y)
                    {
                        runSlides(0.8, HIGHEST_SLIDE_HEIGHT);
                    }

                    if (gamepad2.a)
                    {
                        runSlides(0.8, LOWEST_SLIDE_HEIGHT);
                    }

                    if (gamepad2.b)
                    {
                        specimenClawGrab.setPosition(1);
                    }

                    if (gamepad2.dpad_up)
                    {
                        specimenClawPitch.setPosition(HANSEN_CLAW_MAX_PITCH);
                    }
                    if (gamepad2.dpad_right)
                    {
                        specimenClawPitch.setPosition(HANSEN_CLAW_MEDIUM_PITCH);
                    }
                    if(gamepad2.dpad_down)
                    {
                        specimenClawPitch.setPosition(HANSEN_CLAW_MIN_PITCH);
                    }

                    if(gamepad2.left_bumper)
                    {
                        currState = states.START;
                    }

                    break;
                case BINTAKE:
                    runSlides(0.8, bintakeTolorance);

                    if(gamepad2.dpad_up)
                    {
                        bintakeJoint.setPosition(BINTAKE_DOWN);
                    }

                    if(gamepad2.left_bumper)
                    {
                        bintake.setPower(-1);
                    }

                    if(gamepad2.dpad_down)
                    {
                        currState = states.DEPLOY;
                    }
                    break;
                case DEPLOY:
                    bintake.setPower(1);
                    sleep(100);
                    bintake.setPower(0);
                    bintakeJoint.setPosition(BINTAKE_UP);

                    sleep (100);

                    runSlides(0.8, LOWEST_SLIDE_HEIGHT);

                    currState = states.START;
                    break;
                case HANG:
                    if (gamepad2.y)
                    {
                        runSlides(0.8, HIGHEST_SLIDE_HEIGHT);
                    }

                    if(gamepad2.a) {
                        rLiftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        lLiftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        rLiftSlide.setPower(-1);
                        lLiftSlide.setPower(-1);
                    }
                    break;

            }




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

    public void runSlides(double power, int target)
    {

        int err = Math.abs(rLiftSlide.getCurrentPosition() - lLiftSlide.getCurrentPosition());
        int max = 1;
        if (power < 0) {
            max = -1;
        } else if (power == 0) {
            max = 0;
        }


        rLiftSlide.setTargetPosition(target);
        lLiftSlide.setTargetPosition(target);
        rLiftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lLiftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (power == 0) {
            rLiftSlide.setPower(power);
            lLiftSlide.setPower(power);
            return;
        }

        while (err >= 50) {
            if (rLiftSlide.getCurrentPosition() < lLiftSlide.getCurrentPosition()) {
                rLiftSlide.setPower(max);
                lLiftSlide.setPower(power);
            } else if (lLiftSlide.getCurrentPosition() < rLiftSlide.getCurrentPosition()) {
                lLiftSlide.setPower(max);
                rLiftSlide.setPower(power);
            } else {
                rLiftSlide.setPower(max);
                lLiftSlide.setPower(max);
            }
            err = Math.abs(rLiftSlide.getCurrentPosition() - lLiftSlide.getCurrentPosition());

            if(gamepad1.b) {
                return;
            }
        }

        rLiftSlide.setPower(max);
        lLiftSlide.setPower(max);
    }
}
