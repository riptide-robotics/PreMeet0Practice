package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ExportAprilTagLibraryToBlocks;

@TeleOp(name = "Intake Spin")
public class IntakeSpin extends LinearOpMode {


    CRServo intakeWheels1;
    CRServo intakeWheels2;

    public void runOpMode() throws InterruptedException{
        intakeWheels1 = hardwareMap.crservo.get("intakeServo1");
        intakeWheels2 = hardwareMap.crservo.get("intakeServo2");



        waitForStart();
        while(opModeIsActive()){

            if (gamepad1.dpad_right)
            {
                intakeWheels1.setPower(0);
                intakeWheels2.setPower(0);
            }

            if (gamepad1.left_bumper)
            {
                intakeWheels1.setPower(1);
                intakeWheels2.setPower(1);
            }

            if(gamepad1.right_bumper)
            {
                intakeWheels1.setPower(-1);
                intakeWheels2.setPower(-1);
            }


        }
    }
}
