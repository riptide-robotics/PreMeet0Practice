package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "servoPositionConfig", group = "testing")
public class sdfalj extends LinearOpMode {

    public static double pitchpos = 1;
    public static double grabpos = 0.5;

    public Servo specimenClawGrab;
    public Servo specimenClawPitch;

    @Override
    public void runOpMode() throws InterruptedException {
        specimenClawGrab = hardwareMap.servo.get("bintakeJoint");
        specimenClawPitch = hardwareMap.servo.get("specimenPitch");

        waitForStart();

        while(opModeIsActive())
        {
            specimenClawGrab.setPosition(grabpos);
            specimenClawPitch.setPosition(pitchpos);
        }


    }

}
