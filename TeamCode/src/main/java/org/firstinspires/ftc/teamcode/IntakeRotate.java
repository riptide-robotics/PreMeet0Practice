package org.firstinspires.ftc.teamcode;

import androidx.annotation.ChecksSdkIntAtLeast;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Intake extend/retract")
public class IntakeRotate extends LinearOpMode {


    public static double rotateUp = 0; // IDK THE VALUE
    public static double rotateDown = 0; // IDK THE VALUE



    Servo lServo;
    Servo rServo;


    @Override
    public void runOpMode() throws InterruptedException{
        lServo = hardwareMap.servo.get("lIntakeServo");
        rServo = hardwareMap.servo.get("rIntakeServo");

        waitForStart();
        while(opModeIsActive())
        {
            if(gamepad1.dpad_down)
            {
                lServo.setPosition(rotateDown);
                rServo.setPosition(rotateUp);
                telemetry.addData("Intake Slides: ", "going in");
                telemetry.update();
            }

            if(gamepad1.dpad_up)
            {
                lServo.setPosition(rotateUp);
                rServo.setPosition(rotateDown);
                telemetry.addData("Intake Slides: ", "going out");
                telemetry.update();
            }

            if(gamepad1.left_bumper)
            {
                lServo.setPosition(lServo.getPosition());
                rServo.setPosition(rServo.getPosition());
                sleep(100);
                telemetry.addData("Intake Slides: ", "frozen (in theory this should work)");
                telemetry.update();
            }
        }
    }
}
