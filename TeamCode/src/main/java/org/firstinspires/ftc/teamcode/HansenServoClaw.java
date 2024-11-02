package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "SpecimanClawtest", group = "testing")
public class HansenServoClaw extends LinearOpMode {

    //FURTHEST GRAB SERVO CAN GO IS 0.7!!!!!!!!!!!!!!!! grab 1 is open
    // roughly 0.61 is sweet spot for pitch. pitch 1 is down
    //pitch close is 0.1
    public static double clawPositionOpen = 1;
    public static double clawPositionClose = 0.8;
    public static double pitchPositionUp = 0.5;
    public static double pitchPositionDown = 0.8;

    public double pitches[] = {0.5, 0.61};
    public int i = 1;


    public Servo specimenClawGrab;
    public Servo specimenClawPitch;

    @Override
    public void runOpMode() throws InterruptedException {
        specimenClawGrab = hardwareMap.servo.get("specimenGrab");
        specimenClawPitch = hardwareMap.servo.get("specimenPitch");

        specimenClawPitch.setPosition(0.61);
        specimenClawGrab.setPosition(1);

        waitForStart();

        while(opModeIsActive())
        {
            if(gamepad1.right_trigger > 0)
            {
                pitchDown();
            }

            if(gamepad1.right_bumper)
            {
                pitchUp();
            }

            if (gamepad1.b)
            {
                specimenClawGrab.setPosition(clawPositionOpen);
            }

            if (gamepad1.x)
            {
                specimenClawGrab.setPosition(clawPositionClose);
            }

            telemetry.addData("trigervalue", gamepad1.right_trigger);
            telemetry.update();
        }

        if(isStopRequested())
        {
            specimenClawPitch.setPosition(0.1);
        }


    }

    public void pitchUp() throws InterruptedException {
        if (i == 0) {return;}

        i--;

        specimenClawPitch.setPosition(pitches[i]);

    }

    public void pitchDown() throws InterruptedException {
        if (i == 1) {return;}

        i++;

        specimenClawPitch.setPosition(pitches[i]);

    }
}
