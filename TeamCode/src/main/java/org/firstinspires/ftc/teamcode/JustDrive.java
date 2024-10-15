package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "justDrive", group = "Meet0")
public class JustDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // When init button is hit
        DriveTrain wheels = new DriveTrain();

        waitForStart();
        if (isStopRequested()) return;

        // All code here is after the start button is hit.
        // Main loop; until stop button is hit
        while(opModeIsActive()){
            wheels.fieldCentricDrive();
        }
    }
}
