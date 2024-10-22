package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "justDrive", group = "Meet0")
public class JustDrive extends LinearOpMode {

    private  DcMotor frWheel, flWheel, brWheel, blWheel;

    private final int LOW_BUCKET = 1000;
    private final int RETRACTED = 100;

    @Override
    public void runOpMode() throws InterruptedException {

        // When init button is hit

        DriveTrain wheels = new DriveTrain(hardwareMap, gamepad1);

        waitForStart();
        if (isStopRequested()) return;

        // All code here is after the start button is hit.
        // Main loop; until stop button is hit
        while(opModeIsActive()){

            wheels.fieldCentricDrive();
        }
    }
}
