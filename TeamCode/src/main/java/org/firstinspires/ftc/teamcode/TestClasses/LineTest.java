package org.firstinspires.ftc.teamcode.TestClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Autonomous.Pathing.TrapezoidalMotionProfile;
import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "line test")
public class LineTest extends LinearOpMode {

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

            double currPos = robot.getCurrPos().getY(DistanceUnit.CM);
            double err = expectedPos - currPos;
            double output = pidController.calculate(err);

            robot.setWheelPowers(output, output, output, output);
        }
    }
}
