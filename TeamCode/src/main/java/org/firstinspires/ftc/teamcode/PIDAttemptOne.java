package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Slides Test 1", group = "PID")
public class PIDAttemptOne extends LinearOpMode {

    DcMotor lLiftSlide;
    DcMotor rLiftSlide;

    @Override
    public void runOpMode() throws InterruptedException {
        lLiftSlide = hardwareMap.dcMotor.get("lSlide");
        rLiftSlide = hardwareMap.dcMotor.get("rSlide");

        lLiftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rLiftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lLiftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rLiftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lLiftSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        final int COUNTS_PER_REVOLUTION = 752;

        final int highestHigh = 2000;
        final int lowestLow = 100;

        waitForStart();

        while(opModeIsActive())
        {
            if(gamepad1.y)
            {
                runSlides(1, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if(gamepad1.b)
            {
                runSlides(0, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            
            if(gamepad1.x)
            {
                //code
            }
            
            if(gamepad1.a)
            {
                runSlides(-1, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            telemetry.addData("Left Slide current Encoder tick:", lLiftSlide.getCurrentPosition());
            telemetry.addData("Left Side current e=Encoder tick:", rLiftSlide.getCurrentPosition());

            telemetry.update();
        } 

    }

    public void runSlides(double power, DcMotor.RunMode runmode)
    {

        //we read encoder tick values

        //if |err| >= 50 encoder ticks

        // apply a multiplier * some value to bring the encoders back to equal
        rLiftSlide.setMode(runmode);
        rLiftSlide.setPower(power);
        
        lLiftSlide.setMode(runmode);
        lLiftSlide.setPower(power);
    }


}
