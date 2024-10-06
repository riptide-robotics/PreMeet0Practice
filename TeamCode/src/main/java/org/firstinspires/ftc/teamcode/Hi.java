package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Hi extends LinearOpMode {

    private DcMotor kingMotor;



    @Override
    public void runOpMode() throws InterruptedException {
        //Boot the robot up


        kingMotor = hardwareMap.dcMotor.get("kingMotor");

        kingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        kingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while(opModeIsActive()) {
            if(gamepad1.dpad_up)
            {
                kingMotor.setPower(0.5);
            }
            if(gamepad1.dpad_down)
            {
                kingMotor.setPower(-0.5);
            }
        }

    }

}