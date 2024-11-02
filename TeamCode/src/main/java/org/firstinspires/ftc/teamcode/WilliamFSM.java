package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Meet 1 William Finite State Machine")
public class WilliamFSM extends LinearOpMode {
    //----------All motors----------
    //Field Centric Drive
    DcMotor flWheelMotor;
    DcMotor frWheelMotor;
    DcMotor brWheelMotor;
    DcMotor blWheelMotor;
    //Hang
    DcMotor rSlideMotor;
    DcMotor lSlideMotor;

    //----------All servos----------
    //Linkage
    CRServo crServoLeft;
    CRServo crServoRight;

    Servo extendServoLeft;
    Servo extendServoRight;
    Servo intakePitchServo;
    Servo lIntakeAngleServo;
    Servo rIntakeAngleServo;
    //Hang
    Servo rSlideJoint;
    Servo lSlideJoint;
    //Outtake
    Servo outtakeServo;

    //----------All variables----------
    //Field Centric Drive vars
    private double x;
    private double y;
    private double rx;
    private int kpx; //proportional constant x
    private int kdx; //derivative constant x
    private int kix; //integral constant x

    private int kpy;
    private int kdy;
    private int kiy;

    private double previousErrorx = 0;
    private double previousErrory = 0;

    private double xpos = 0;
    private double ypos = 0;
    private double theta = 0;

    private double integralx = 0;
    private double integraly = 0;
    private ElapsedTime time = new ElapsedTime();

    private final double R = 4.8;
    private final int V = 2000;
    private final double C = 1/V; // Circumference
    private final double L = 10; //Unknown, put a random number; distance between odomitors
    private final double B = 0/*Unknown*/;

    //Linkage vars

    private final double minExtend = 0 /*Unknown*/;
    private final double maxExtend = 0 /*Unknown*/;
    private final double minPitch = 0 /*Unknown*/;
    private final double maxPitch = 0 /*Unknown*/;
    private final double minAngle = 0 /*Unknown*/;
    private final double maxAngle = 0 /*Unknown*/;

    private final double minOut = 0/*Unknown*/;
    private final double maxOut = 0/*Unknown*/;

    //Hang vars

    public static double rotateUp = 0.7;
    public static double rotateDown = 0.3;
    public static double rotateMiddle = 0.5;

    public static int highestPos = 2500;
    public static int lowestPos = 100;

    private double rServoPos = 0;
    private double lServoPos = 0;

    public static double hangPos = 0.4;

    //----------Miscellaneous----------

    private IMU imu;

    //----------States----------

    public states currentState;
    public enum states {
        CLEANUP,
        DRIVE,
        HANG,
        LINKAGE,
        OUTTAKE
    }

    public void runOpMode() {
        // Init of all variables
        crServoLeft = hardwareMap.crservo.get("lCrServo"); // configure this
        crServoRight = hardwareMap.crservo.get("rCrServo");

        extendServoLeft = hardwareMap.servo.get("lExtend");
        extendServoRight = hardwareMap.servo.get("rExtend");
        intakePitchServo = hardwareMap.servo.get("pitchServo");
        lIntakeAngleServo = hardwareMap.servo.get("lIntakeAngle");
        rIntakeAngleServo = hardwareMap.servo.get("rIntakeAngle");

        //-----------------------------------
        flWheelMotor = hardwareMap.dcMotor.get("flWheel");
        frWheelMotor = hardwareMap.dcMotor.get("frWheel");
        brWheelMotor = hardwareMap.dcMotor.get("brWheel");
        blWheelMotor = hardwareMap.dcMotor.get("blWheel");

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

        imu.resetYaw();

        imu.initialize(parameters);

        flWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //-----------------------------------------------------------

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

        //--------------------------------------------------------------------

        outtakeServo = hardwareMap.servo.get("outtakeServo");

        waitForStart();

        flWheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        blWheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        currentState = states.CLEANUP;

        while(opModeIsActive()) {
            switch(currentState) {
                case CLEANUP:
                    //Linkage vars
                    if(gamepad1.b) {
                        currentState = states.DRIVE;
                    }

                    extendServoLeft.setPosition(minExtend);
                    extendServoRight.setPosition(minExtend);
                    lIntakeAngleServo.setPosition(minAngle);
                    rIntakeAngleServo.setPosition(minAngle);
                    intakePitchServo.setPosition(minPitch);

                    //Hang vars
                    rSlideJoint.setPosition(rotateDown);
                    lSlideJoint.setPosition(rotateUp);

                    rSlideMotor.setTargetPosition(lowestPos);
                    lSlideMotor.setTargetPosition(lowestPos);

                    break;
                case DRIVE:
                    fieldDrive();
                    break;
                case HANG:
                    if (gamepad1.x) {
                        currentState = states.DRIVE;
                    }

                    double rSlideCurrentPos = rSlideJoint.getPosition();
                    double lSlideCurrentPos = lSlideJoint.getPosition();
                    // rotate up
                    if (gamepad2.dpad_left)
                    {
                        rSlideJoint.setPosition(rotateUp);
                        lSlideJoint.setPosition(rotateDown);
                        telemetry.addData("Slides: ", "rotating up");
                        telemetry.update();
                    }

                    // rotate down
                    if (gamepad2.dpad_right)
                    {
                        rSlideJoint.setPosition(rotateDown);
                        lSlideJoint.setPosition(rotateUp);
                        telemetry.addData("Slides: ", "rotating down");
                        telemetry.update();
                    }

                    // Both slides up
                    if (gamepad2.dpad_up)
                    {
                        //moveSlides(1, highestPos);
                        rSlideMotor.setPower(1);
                        lSlideMotor.setPower(1);
                        telemetry.addData("Slides: ", "moving up");
                        telemetry.update();
                    }

                    // Both slides down
                    if (gamepad2.dpad_down)
                    {
                        //moveSlides(-1, lowestPos);
                        rSlideMotor.setPower(-1);
                        lSlideMotor.setPower(-1);
                        telemetry.addData("Slides: ", "moving down");
                        telemetry.update();
                    }

                    // Both slides stopped
                    if (gamepad2.right_bumper)
                    {
                        rSlideMotor.setPower(0);
                        lSlideMotor.setPower(0);
                        telemetry.addData("Slides: ", "stopped");
                        telemetry.update();
                    }
                    // Rotation stopped
                    if (gamepad2.x)
                    {
                        //slideJoint1.setPosition(hangPos);
                        //slideJoint2.setPosition(1 - hangPos);
                        rSlideJoint.setPosition(rSlideCurrentPos - rServoPos); // IDK how to pause servo position help
                        lSlideJoint.setPosition(lServoPos - lSlideCurrentPos); // IDK how to pause servo position help
                        sleep(3000);
                        telemetry.addData("Rotation Servo Right: ", rSlideCurrentPos - rServoPos);
                        telemetry.addData("Rotation Servo Left: ", lServoPos - lSlideCurrentPos);
                        telemetry.update();
                    }

                    // Reset Rotation
                    if (gamepad2.left_bumper)
                    {
                        rSlideJoint.setPosition(rotateMiddle);
                        lSlideJoint.setPosition(rotateMiddle);
                        telemetry.addData("Rotation: ", "Reset");
                        telemetry.update();
                    }

                    rServoPos = rSlideJoint.getPosition();
                    lServoPos = lSlideJoint.getPosition();

                    fieldDrive();

                    break;
                case LINKAGE:

                    if (gamepad1.x) {
                        currentState = states.DRIVE;
                    }

                    if(gamepad2.b) {
                        crServoLeft.setPower(0);
                        crServoRight.setPower(0);
                    } else if(gamepad2.left_bumper) {
                        crServoLeft.setPower(1);
                        crServoRight.setPower(-1);
                    } else if(gamepad2.right_bumper) {
                        crServoLeft.setPower(-1);
                        crServoRight.setPower(1);
                    }

                    if(gamepad2.dpad_down) {
                        extendServoLeft.setPosition(minExtend);
                        extendServoRight.setPosition(minExtend);
                    }

                    if(gamepad2.dpad_up) {
                        extendServoLeft.setPosition(maxExtend);
                        extendServoRight.setPosition(maxExtend);
                    }

                    if(gamepad2.dpad_left) {
                        lIntakeAngleServo.setPosition(minAngle);
                        rIntakeAngleServo.setPosition(minAngle);
                    }

                    if(gamepad2.dpad_right) {
                        lIntakeAngleServo.setPosition(maxAngle);
                        rIntakeAngleServo.setPosition(maxAngle);
                    }

                    if(gamepad2.left_trigger != 0) {
                        intakePitchServo.setPosition(minPitch);
                    }

                    if(gamepad2.right_trigger != 0) {
                        intakePitchServo.setPosition(maxPitch);
                    }

                    fieldDrive();

                    break;
                case OUTTAKE:
                    if(gamepad2.y) {
                        outtakeServo.setPosition(maxOut);
                    } else if (gamepad2.a) {
                        outtakeServo.setPosition(minOut);
                    }
                    fieldDrive();
                    break;
            }
        }
    }

    public void fieldDrive() {
        rSlideJoint.setPosition(rotateMiddle);
        lSlideJoint.setPosition(rotateMiddle);

        if(currentState == states.DRIVE) {
            if (gamepad1.y) {
                currentState = states.HANG;
            } else if (gamepad1.x) {
                currentState = states.CLEANUP;
            } else if (gamepad1.b) {
                currentState = states.LINKAGE;
            }
        }

        //controller values inputted
        y = -gamepad1.left_stick_y;
        //for margin of error in movement
        x = gamepad1.left_stick_x * 1.1;
        rx = gamepad1.right_stick_x;

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        //power set for each of our motos
        double flpower = (rotY - rotX + rx)/denominator;
        double frpower = (rotY + rotX - rx)/denominator;
        double blpower = (rotY + rotX + rx)/denominator;
        double brpower = (rotY - rotX - rx)/denominator;

        //setting the power of each of our motos
        flWheelMotor.setPower(flpower);
        frWheelMotor.setPower(frpower);
        brWheelMotor.setPower(blpower);
        blWheelMotor.setPower(brpower);

        telemetry.addData("X position: ", xpos);
        telemetry.update();

        telemetry.addData("Y position: ", ypos);
        telemetry.update();

        telemetry.addData("Theta: ", theta);
        telemetry.update();
    }
}