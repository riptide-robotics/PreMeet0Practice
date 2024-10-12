package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Slides Test 1", group = "PID")
public class PIDAttemptOne extends LinearOpMode {

    DcMotor lLiftSlide;
    DcMotor rLiftSlide;

    final int COUNTS_PER_REVOLUTION = 752;

    final int highestHigh = 2350; // important
    final int lowestLow = 100; // important

    @Override
    public void runOpMode() throws InterruptedException {
        lLiftSlide = hardwareMap.dcMotor.get("lSlide");
        rLiftSlide = hardwareMap.dcMotor.get("rSlide");

        lLiftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rLiftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lLiftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rLiftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lLiftSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive())
        {
            if(gamepad1.y)
            {
                runSlides(0.85, highestHigh);
            }

            if(gamepad1.b)
            {
                runSlides(0, 500);
            }
            
            if(gamepad1.x)
            {
                //code
            }
            
            if(gamepad1.a)
            {
                runSlides(-0.85, lowestLow);
            }

            if(gamepad1.dpad_down) {
                rLiftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lLiftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rLiftSlide.setPower(-1);
                lLiftSlide.setPower(-1);
            }

            telemetry.addData("Left Slide current Encoder tick:", lLiftSlide.getCurrentPosition());
            telemetry.addData("Left Side current e=Encoder tick:", rLiftSlide.getCurrentPosition());

            telemetry.update();
        } 

    }

    public void runSlides(double power, int target)
    {

        //we read encoder tick values

        //if |err| >= 50 encoder ticks

        // apply a multiplier * some value to bring the encoders back to equal

        //rLiftSlide.setTargetPosition(highestHigh);
        //rLiftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //rLiftSlide.setPower(1);

        //rLiftSlide.getCurrentPosition();

        int err = Math.abs(rLiftSlide.getCurrentPosition() - lLiftSlide.getCurrentPosition());
        int max = 1;
        if (power < 0) {
            max = -1;
        } else if (power == 0) {
            max = 0;
        }
        rLiftSlide.setTargetPosition(target);
        lLiftSlide.setTargetPosition(target);
        rLiftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lLiftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (power == 0) {
            rLiftSlide.setPower(power);
            lLiftSlide.setPower(power);
            return;
        }

        while (err >= 50) {
            if (rLiftSlide.getCurrentPosition() < lLiftSlide.getCurrentPosition()) {
                rLiftSlide.setPower(max);
                lLiftSlide.setPower(power);
            } else if (lLiftSlide.getCurrentPosition() < rLiftSlide.getCurrentPosition()) {
                lLiftSlide.setPower(max);
                rLiftSlide.setPower(power);
            } else {
                rLiftSlide.setPower(max);
                lLiftSlide.setPower(max);
            }
            err = Math.abs(rLiftSlide.getCurrentPosition() - lLiftSlide.getCurrentPosition());

            if(gamepad1.b) {
                return;
            }
        }

        rLiftSlide.setPower(max);
        lLiftSlide.setPower(max);
    }


}
