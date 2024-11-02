package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Intake Spin")
public class IntakeSpin extends LinearOpMode {


    CRServo intakeWheels1;
    CRServo intakeWheels2;
    Servo lSlideServo;
    Servo rSlideServo;
    Servo intakePivotServo;
    Servo rIntakeArmPivot;
    Servo lIntakeArmPivot;

    public static double extendSlidesOut = 0;    // IDK THE VALUE
    public static double extendSlidesIn = 0;  // IDK THE VALUE

    public static double pivotIntakeUp = 0; // IDK THE VALUE
    public static double pivotIntakeDown = 0; // IDK THE VALUE

    public static double pivotIntakeArmUp = 0; // IDK THE VALUE
    public static double pivotIntakeArmDown = 0; // IDK THE VALUE




    public void runOpMode() throws InterruptedException{

        intakeWheels1 = hardwareMap.crservo.get("intakeServo1");
        intakeWheels2 = hardwareMap.crservo.get("intakeServo2");

        lSlideServo = hardwareMap.servo.get("lSlideServo");
        rSlideServo = hardwareMap.servo.get("rSlideServo");

        intakePivotServo = hardwareMap.servo.get("intakePivotServo");

        rIntakeArmPivot = hardwareMap.servo.get("rArmPivot");
        lIntakeArmPivot = hardwareMap.servo.get("lArmPivot");



        waitForStart();
        while(opModeIsActive()){


            // INTAKE SPIN
            //----------------------------------------------------------------------------------------------------------------
            if (gamepad1.b)
            {
                intakeWheels1.setPower(0);
                intakeWheels2.setPower(0);
            }

            if (gamepad1.left_bumper)
            {
                intakeWheels1.setPower(1);
                intakeWheels2.setPower(-1);
            }

            if(gamepad1.right_bumper)
            {
                intakeWheels1.setPower(-1);
                intakeWheels2.setPower(1);
            }

            //----------------------------------------------------------------------------------------------------------------

            // INTAKE EXTEND/RETRACT

            //----------------------------------------------------------------------------------------------------------------

            if(gamepad1.right_trigger > 0)
            {
                lSlideServo.setPosition(extendSlidesIn);
                rSlideServo.setPosition(extendSlidesOut);
                telemetry.addData("Intake Slides: ", "going in");
                telemetry.update();
            }

            if(gamepad1.left_trigger > 0)
            {
                lSlideServo.setPosition(extendSlidesOut);
                rSlideServo.setPosition(extendSlidesIn);
                telemetry.addData("Intake Slides: ", "going out");
                telemetry.update();
            }

            // STOP INTAKE EXTEND/RETRACT
            if(gamepad1.x)
            {
                lSlideServo.setPosition(lSlideServo.getPosition());
                rSlideServo.setPosition(rSlideServo.getPosition());
                sleep(100);
                telemetry.addData("Intake Slides: ", "frozen (in theory this should work)");
                telemetry.update();
            }

            //-----------------------------------------------------------------------------------------------------------------

            // INTAKE PIVOT

            //----------------------------------------------------------------------------------------------------------------

            if (gamepad1.dpad_up)
            {
                intakePivotServo.setPosition(pivotIntakeUp);
                telemetry.addData("Intake: ", "rotating up");
                telemetry.update();
            }

            if (gamepad1.dpad_down)
            {
                intakePivotServo.setPosition(pivotIntakeDown);
                telemetry.addData("Intake: ", "rotating down");
                telemetry.update();
            }

            if (gamepad1.y)
            {
                rIntakeArmPivot.setPosition(pivotIntakeArmUp);
                lIntakeArmPivot.setPosition(pivotIntakeArmDown);
                telemetry.addData("Intake Arm: ", "rotating up");
                telemetry.update();
            }

            if (gamepad1.a)
            {
                rIntakeArmPivot.setPosition(pivotIntakeArmDown);
                lIntakeArmPivot.setPosition(pivotIntakeArmUp);
                telemetry.addData("Intake Arm: ", "rotating down");
                telemetry.update();
            }
        }
    }
}
