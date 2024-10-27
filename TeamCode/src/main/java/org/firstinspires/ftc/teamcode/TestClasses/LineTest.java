package org.firstinspires.ftc.teamcode.TestClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.Pathing.TrapezoidalMotionProfile;
import org.firstinspires.ftc.teamcode.PIDController;

public class LineTest extends LinearOpMode {
    PIDController pidController = new PIDController(0.7, 0.1, 0.4);
    double endPos = 0.0;
    TrapezoidalMotionProfile trapezoidalMotionProfile = new TrapezoidalMotionProfile(30, 53);

    @Override
    public void runOpMode() throws InterruptedException {
        trapezoidalMotionProfile.calculateProfile(endPos);

        waitForStart();
        pidController.reset();

        double exPos;

        while(opModeIsActive()) {
            exPos = trapezoidalMotionProfile.getExpectedPosition(pidController.getTime());

        }
    }
}
