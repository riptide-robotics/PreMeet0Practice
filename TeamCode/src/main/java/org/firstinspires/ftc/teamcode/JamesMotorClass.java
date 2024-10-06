package org.firstinspires.ftc.teamcode;
// importing stuff
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

// Teleop name
@TeleOp(name = "TeleOp")
public class JamesMotorClass extends LinearOpMode {
    DcMotor kingMotor;
    @Override

    public void runOpMode() throws InterruptedException{
        // referencing motor so program knows what is is
        kingMotor = hardwareMap.dcMotor.get("kingMotor");

        // Reset localization for distance of motor
        kingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        kingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        while(opModeIsActive())
        {

            if (gamepad1.dpad_down)
            {
                kingMotor.setPower(0.5);

            }

            if(gamepad1.b)
            {
                kingMotor.setPower(0);
            }

            if(gamepad1.dpad_up)
            {
                kingMotor.setPower(-0.5);
            }

            // For joystick
            kingMotor.setPower(gamepad1.left_stick_y);
        }




    }




}
