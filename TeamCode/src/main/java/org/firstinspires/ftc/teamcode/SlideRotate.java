package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Slide Move")
public class SlideRotate extends LinearOpMode {
    Servo slideJoint;
    final double rotateUp = 0; //idk the value
    final double rotateDown = 0; // idk the value
    @Override



    public void runOpMode() throws InterruptedException{
        slideJoint = hardwareMap.servo.get("slideJoint");

        waitForStart();
        while(opModeIsActive()){

            if (gamepad1.dpad_down)
            {
                slideJoint.setPosition(rotateDown);
            }

            if (gamepad1.dpad_up)
            {
                slideJoint.setPosition(rotateUp);
            }

            if (gamepad1.dpad_left)
            {
                slideJoint.setPosition(0);
            }


        }
    }
}
