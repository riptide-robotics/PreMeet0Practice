package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = "MEET 1 CONTROLS SELECT ME")
public class Meet1Controller extends LinearOpMode {

    // INTAKE
    CRServo intakeWheelsL;
    CRServo intakeWheelsR;
    Servo lSlideServo;
    Servo rSlideServo;
    Servo intakePivotServo;
    Servo rIntakeArmPivot;
    DcMotor frWheel, flWheel, brWheel, blWheel;


    public static double extendSlidesOut = 0.7;    // IDK THE VALUE
    public static double extendSlidesIn = 0.5;  // IDK THE VALUE

    public static double pivotIntakeUp = 0.5; // IDK THE VALUE
    public static double pivotIntakeDown = 0.3; // IDK THE VALUE

    public static double pivotIntakeArmUp = 0.6; // IDK THE VALUE
    public static double pivotIntakeArmDown = 0.4; // IDK THE VALUE






    // HANG

    // Right slides
    Servo rSlideJoint;
    DcMotor rSlideMotor;

    IMU imu;

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











    public states currentState;
    public enum states{
        RESET,
        INTAKE,
        HANG
    }




    public void runOpMode() throws InterruptedException{

        // INTAKE HARDWARE MAP
        intakeWheelsL = hardwareMap.crservo.get("lCrServo");
        intakeWheelsR = hardwareMap.crservo.get("rCrServo");
        lSlideServo = hardwareMap.servo.get("lExtend");
        rSlideServo = hardwareMap.servo.get("rExtend");
        intakePivotServo = hardwareMap.servo.get("pitchServo");
        rIntakeArmPivot = hardwareMap.servo.get("rIntakeAngle");
        // --------------------------------------------------------------



        // HANG HARDWARE MAP
        rSlideJoint = hardwareMap.servo.get("rSlideJoint");
        lSlideJoint = hardwareMap.servo.get("lSlideJoint");
        rSlideMotor = hardwareMap.dcMotor.get("rSlide");
        lSlideMotor = hardwareMap.dcMotor.get("lSlide");
        //----------------------------------------------------------------

        // REVERSE LEFT SIDE
        lSlideServo.setDirection(Servo.Direction.REVERSE);
        intakeWheelsL.setDirection(CRServo.Direction.REVERSE);
        lSlideMotor.setDirection(DcMotor.Direction.REVERSE);
        //----------------------------------------------------------


        // SET LOWEST POSITION TO SLIDES WHEN INITIALIZING
        rSlideJoint.setPosition(rotateDown);
        lSlideJoint.setPosition(rotateUp);

        rSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // driving
        frWheel = hardwareMap.dcMotor.get("frWheel");
        flWheel = hardwareMap.dcMotor.get("flWheel");
        brWheel = hardwareMap.dcMotor.get("brWheel");
        blWheel = hardwareMap.dcMotor.get("blWheel");

        blWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        frWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        brWheel.setDirection(DcMotorSimple.Direction.FORWARD);

        brWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        blWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();
        while(opModeIsActive()) {


            switch (currentState) {
                case INTAKE:

                    fieldCentricDrive();

                    // INTAKE SPIN
                    //----------------------------------------------------------------------------------------------------------------
                    if (gamepad2.b) {
                        intakeWheelsL.setPower(0);
                        intakeWheelsR.setPower(0);
                    }

                    if (gamepad2.left_bumper) {
                        intakeWheelsL.setPower(1);
                        intakeWheelsR.setPower(1);
                    }

                    if (gamepad2.right_bumper) {
                        intakeWheelsL.setPower(-1);
                        intakeWheelsR.setPower(1);
                    }

                    //----------------------------------------------------------------------------------------------------------------

                    // INTAKE EXTEND/RETRACT

                    //----------------------------------------------------------------------------------------------------------------

                    if (gamepad2.right_trigger > 0) {
                        lSlideServo.setPosition(extendSlidesOut);
                        rSlideServo.setPosition(extendSlidesOut);
                        telemetry.addData("Intake Slides: ", "going in");
                        telemetry.update();
                    }

                    if (gamepad2.left_trigger > 0) {
                        lSlideServo.setPosition(extendSlidesIn);
                        rSlideServo.setPosition(extendSlidesIn);
                        telemetry.addData("Intake Slides: ", "going out");
                        telemetry.update();
                    }

                    //INTAKE EXTEND/RETRACT
                    if (gamepad2.x) {
                        currentState = states.RESET;
                    }

                    //-----------------------------------------------------------------------------------------------------------------

                    // INTAKE PIVOT

                    //----------------------------------------------------------------------------------------------------------------

                    if (gamepad2.dpad_up) {
                        intakePivotServo.setPosition(pivotIntakeUp);
                        telemetry.addData("Intake: ", "rotating up");
                        telemetry.update();
                    }

                    if (gamepad2.dpad_down) {
                        intakePivotServo.setPosition(pivotIntakeDown);
                        telemetry.addData("Intake: ", "rotating down");
                        telemetry.update();
                    }

                    if (gamepad2.y) {
                        rIntakeArmPivot.setPosition(pivotIntakeArmUp);
                        telemetry.addData("Intake Arm: ", "rotating up");
                        telemetry.update();
                    }

                    if (gamepad2.a) {
                        rIntakeArmPivot.setPosition(pivotIntakeArmDown);
                        telemetry.addData("Intake Arm: ", "rotating down");
                        telemetry.update();
                    }

                case RESET:

                    fieldCentricDrive();

                    if (gamepad2.left_bumper)
                    {
                        lSlideServo.setPosition(0);
                        rSlideJoint.setPosition(0);

                        intakePivotServo.setPosition(0);
                        rIntakeArmPivot.setPosition(0);

                        rSlideJoint.setPosition(rotateMiddle);
                        lSlideJoint.setPosition(rotateMiddle);
                    }

                    if (gamepad2.x)
                    {
                        currentState = states.HANG;
                    }

                case HANG:

                    fieldCentricDrive();

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

                    if (gamepad1.x)
                    {
                        currentState = states.RESET;
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
    }

    public void fieldCentricDrive() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1; // 1.1 is to account for hardware inconsistencies.
        double rx = gamepad1.right_stick_x;

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

        imu.resetYaw();
        imu.initialize(parameters);

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); // heading of bot in radians



        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading); // Linear transformations yay
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1); // we like our drivers to have more control
        double frWheelPower = (rotY - rotX - rx) / denominator;
        double flWheelPower = (rotY + rotX + rx) / denominator;
        double brWheelPower = (rotY + rotX - rx) / denominator;
        double blWheelPower = (rotY - rotX + rx) / denominator;

        flWheel.setPower(flWheelPower);
        frWheel.setPower(frWheelPower);
        blWheel.setPower(blWheelPower);
        brWheel.setPower(brWheelPower);
    }
}