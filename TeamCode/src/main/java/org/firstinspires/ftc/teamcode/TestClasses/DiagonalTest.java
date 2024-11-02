package org.firstinspires.ftc.teamcode.TestClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Autonomous.Pathing.TrapezoidalMotionProfile;
import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.Robot;

public class DiagonalTest extends LinearOpMode { // Edit for diagonal
    PIDController pidController = new PIDController(0.7, 0.1, 0.4);
    double endPos = 0.0;
    TrapezoidalMotionProfile trapezoidalMotionProfile = new TrapezoidalMotionProfile(30, 53);

    @Override
    public void runOpMode() throws InterruptedException {

        trapezoidalMotionProfile.calculateProfile(endPos);

        waitForStart();
        pidController.reset();

        double expectedPos;

        Robot robot = new Robot(hardwareMap);
        robot.startOdometry();

        while(opModeIsActive()) {

            trapezoidalMotionProfile.calculateProfile(endPos);
            expectedPos = trapezoidalMotionProfile.getExpectedPosition(pidController.getTime());
            // getting x & y of expectedPos

            double currX = robot.getCurrPos().getX(DistanceUnit.CM);
            double currY = robot.getCurrPos().getY(DistanceUnit.CM);

            //double currPos = Math.sqrt(Math.pow(currX, 2) + Math.pow(currY, 2));
            double currAngle = Math.atan2(currY, currX);

//            double err = expectedPos - currPos;
//            double output = pidController.calculate(err);
//
//            robot.setWheelPowers(output, -output, output, -output); // What to put into this?
        }
    }
}
