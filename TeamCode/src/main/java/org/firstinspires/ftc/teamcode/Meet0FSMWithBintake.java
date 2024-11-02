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
    public static int LOWEST_SLIDE_HEIGHT = 100;
    final int bintakeTolorance = 2000;

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

        DriveTrain wheels = new DriveTrain(hardwareMap, gamepad1);


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
        specimenClawPitch.setPosition(0.61);
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


                    if(gamepad2.back)
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

                    if(gamepad2.dpad_right){
                        bintake.setPower(0);
                    }

                    if(gamepad2.left_bumper)
                    {
                        bintake.setPower(1);
                    }

                    if(gamepad2.y){
                        bintakeJoint.setPosition(BINTAKE_UP);
                    }

                    if(gamepad2.dpad_down)
                    {
                        currState = states.DEPLOY;
                    }
                    break;
                case DEPLOY:
                    bintake.setPower(-1);
                    sleep(100);
                    bintake.setPower(0);
                    bintakeJoint.setPosition(BINTAKE_UP);

                    sleep (500);

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

            handleDriving(2, wheels);




        }

    }

    public void handleDriving(int mode, DriveTrain wheels) throws IllegalArgumentException {
        if (mode == 1) {
            wheels.robotCentricDrive();
        }
        else if(mode == 2) {
            wheels.fieldCentricDrive();
        }
        else {
            throw new IllegalArgumentException("Not a valid Drive Mode: " + mode);
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
