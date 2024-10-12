package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Meet 0 Auton", group = "meet 0")
public class Meet0AutonBlue extends LinearOpMode{
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
    public static int LOWEST_SLIDE_HEIGHT = 100;
    final int bintakeTolorance = 2000;


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
        specimenClawPitch.setPosition(0.61);
        specimenClawGrab.setPosition(1);

        waitForStart();

        if(opModeIsActive()){
            //Moving the robot forwards
            frWheel.setPower(0.25);
            flWheel.setPower(0.25);
            brWheel.setPower(0.25);
            blWheel.setPower(0.25);
            sleep(2000);
            frWheel.setPower(0);
            flWheel.setPower(0);
            brWheel.setPower(0);
            blWheel.setPower(0);

            //Flipping bintake out
            runSlides(0.8, bintakeTolorance);
            sleep(1000);
            bintakeJoint.setPosition(BINTAKE_DOWN);
            sleep(1000);

            //Deploying bintake out
            bintake.setPower(-1);
            sleep(2000);
            bintake.setPower(0);


            //bringing bintake back in
            bintakeJoint.setPosition(BINTAKE_UP);
            sleep(1000);
            runSlides(0.8, LOWEST_SLIDE_HEIGHT);

            //Backing up robot
            frWheel.setPower(-0.25);
            flWheel.setPower(-0.25);
            brWheel.setPower(-0.25);
            blWheel.setPower(-0.25);
            sleep(1000);
            frWheel.setPower(0);
            flWheel.setPower(0);
            brWheel.setPower(0);
            blWheel.setPower(0);


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

