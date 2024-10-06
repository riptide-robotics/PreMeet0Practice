package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
@TeleOp(name="TEST MOTOR")
public class WilliamsFirstCode extends LinearOpMode {
    DcMotor kingMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        //getting the motor into an object with the motor ID
        kingMotor = hardwareMap.dcMotor.get("kingMotor");

        //For localization of how much the motor has run since it initial starting point
        kingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        kingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        //max value on the dpad
        double max = 0.5;
        while(opModeIsActive()) {
            //when putting the dpad down
            if(gamepad1.dpad_down) {
                //move motor if dpad moved down
                kingMotor.setPower(max);
            }
            if(gamepad1.b) {
                //if B button pressed, motor stops
                kingMotor.setPower(0);
            }
            if(gamepad1.dpad_up) {
                //move motor if dpad moved up
                kingMotor.setPower(-max);
            }
            //set the power of the motor to the y value of the left joy stick
            kingMotor.setPower(-gamepad1.left_stick_y);
        }

    }
}