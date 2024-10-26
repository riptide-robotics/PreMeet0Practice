package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class WilliamFieldCentricDriveTrain extends LinearOpMode {
    DcMotor flWheelMotor;
    DcMotor frWheelMotor;
    DcMotor brWheelMotor;
    DcMotor blWheelMotor;
    private IMU imu;
    private double x;
    private double y;
    private double rx;
    private int kpx; //proportional constant x
    private int kdx; //derivative constant x
    private int kix; //integral constant x

    private int kpy;
    private int kdy;
    private int kiy;

    private double previousErrorx = 0;
    private double previousErrory = 0;

    private double xpos = 0;
    private double ypos = 0;
    private double theta = 0;

    private double integralx = 0;
    private double integraly = 0;
    private ElapsedTime time = new ElapsedTime();

    private final double R = 4.8;
    private final int V = 2000;
    private final double C = 1/V; // Circumference
    private final double L = 10; //Unknown, put a random number; distance between odomitors
    private final double B = 0/*Unknown*/;

    /**
     * This method is constantly running and updating
     * the bot.
     * <p>
     * It returns nothing.
     */

    @Override
    public void runOpMode() throws InterruptedException {
        //Init Motors
        flWheelMotor = hardwareMap.dcMotor.get("flWheel");
        frWheelMotor = hardwareMap.dcMotor.get("frWheel");
        brWheelMotor = hardwareMap.dcMotor.get("brWheel");
        blWheelMotor = hardwareMap.dcMotor.get("blWheel");

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

        imu.resetYaw();

        imu.initialize(parameters);

        //Init all odomitors
        int odomiterParallel1Init = flWheelMotor.getCurrentPosition();
        int odomiterParallel2Init = frWheelMotor.getCurrentPosition();
        int odomiterPerpendicularInit = brWheelMotor.getCurrentPosition();

        waitForStart();

        //Make all motors go in the same direction with same values inputted for power
        //Reverses the direction of the left motors
        flWheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        blWheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //While loop for when controlling robot
        while(opModeIsActive()) {

            //controller values inputted
            y = -gamepad1.left_stick_y;
            //for margin of error in movement
            x = gamepad1.left_stick_x * 1.1;
            rx = gamepad1.right_stick_x;

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

            //power set for each of our motos
            double flpower = (rotY - rotX + rx)/denominator;
            double frpower = (rotY + rotX - rx)/denominator;
            double blpower = (rotY + rotX + rx)/denominator;
            double brpower = (rotY - rotX - rx)/denominator;

            //setting the power of each of our motos
            flWheelMotor.setPower(flpower);
            frWheelMotor.setPower(frpower);
            brWheelMotor.setPower(blpower);
            blWheelMotor.setPower(brpower);

            int odomiterParallel1Change = (flWheelMotor.getCurrentPosition()) - odomiterParallel1Init; //fl odomitor
            int odomiterParallel2Change = (frWheelMotor.getCurrentPosition()) - odomiterParallel2Init; //fr odomitor
            int odomiterPerpendicularChange = (brWheelMotor.getCurrentPosition()) - odomiterPerpendicularInit; // perpen odomitor

            double deltaY = C * (odomiterPerpendicularChange - (B * (odomiterParallel1Change - odomiterParallel2Change)/L));
            double deltaX = C * (odomiterParallel1Change + odomiterParallel2Change)/2;
            double deltaTheta = (C*(odomiterParallel2Change-odomiterParallel1Change))/L;

            deltaX = deltaX*Math.cos(theta) - deltaX*Math.sin(theta);
            deltaY = deltaY*Math.sin(theta) + deltaY*Math.cos(theta);

            xpos += deltaX;
            ypos += deltaY;
            theta += deltaTheta;

            odomiterParallel1Init = flWheelMotor.getCurrentPosition();
            odomiterParallel2Init = frWheelMotor.getCurrentPosition();
            odomiterPerpendicularInit = brWheelMotor.getCurrentPosition();

            telemetry.addData("X position: ", xpos);
            telemetry.update();

            telemetry.addData("Y position: ", ypos);
            telemetry.update();

            telemetry.addData("Theta: ", theta);
            telemetry.update();
        }
    }
    /**
     * This function is to bring the robot smoothly to a
     * set point from its current location on the x axis.
     * It utilizes Proportional, Integral and Derivative functions.
     * <p>
     * This method returns nothing.
     *
     * @param target      the set point of the bot on the x axis
     * @param elapsedTime time elapsed
     */
    public void driveYPID(int target, double elapsedTime) {
        double errory = ypos - target;
        double integralLimity = 1000;
        elapsedTime = time.milliseconds();
        integraly += errory * elapsedTime;

        if (Math.abs(integraly) > integralLimity)
            integraly = Math.signum(integraly) * integralLimity;

        //Derivative part = dError/dt
        double derivativey = (errory - previousErrory) / elapsedTime;
        //Output = P + I + D
        //Output = const * error + const * integral + const * de/dx
        double outputy = errory * kpy + integraly * kiy + kdy * derivativey;

        flWheelMotor.setPower(outputy);
        frWheelMotor.setPower(-outputy);
        brWheelMotor.setPower(outputy);
        blWheelMotor.setPower(-outputy);

        previousErrory = errory;
    }

    /**
     * This function is to bring the robot smoothly to a
     * set point from its current location on the y axis.
     * It utilizes Proportional, Integral and Derivative functions.
     * <p>
     * This method returns nothing.
     *
     * @param target      the set point of the bot on the y axis
     * @param elapsedTime time elapsed
     */
    public void driveXPID(int target, double elapsedTime) {
        double errorx = ypos - target;
        double integralLimitx = 1000;
        elapsedTime = time.milliseconds();
        integralx += errorx * elapsedTime;

        if (Math.abs(integralx) > integralLimitx)
            integralx = Math.signum(integralx) * integralLimitx;

        //Derivative part = dError/dt
        double derivativex = (errorx - previousErrorx) / elapsedTime;
        //Output = P + I + D
        //Output = const * error + const * integral + const * de/dx
        double outputx = errorx * kpy + integraly * kiy + kdy * derivativex;

        flWheelMotor.setPower(outputx);
        frWheelMotor.setPower(-outputx);
        brWheelMotor.setPower(outputx);
        blWheelMotor.setPower(-outputx);

        previousErrorx = errorx;
    }

    /**
     * This function is to bring the robot smoothly to a
     * set point from its current location on the both axis's.
     * It utilizes Proportional, Integral and Derivative functions.
     * <p>
     * This method returns nothing. It calls both the X and Y PID
     * loops.
     *
     * @param xpos x position set point
     * @param ypos y position set point
     */
    public void goTo(int xpos, int ypos) {
        ElapsedTime time = new ElapsedTime();
        while(Math.abs(this.xpos - xpos) > 10 && Math.abs(this.ypos - ypos) > 10) {
            driveYPID(ypos, time.milliseconds());
            driveXPID(xpos, time.milliseconds());

            time.reset();
        }
    }

    /**
     * James Code:
     */
    public void deadWheelDirection() {
        //Get current readings from odomitors
        int flOdometer = flWheelMotor.getCurrentPosition();
        int frOdometer = frWheelMotor.getCurrentPosition();
        int blOdometer = blWheelMotor.getCurrentPosition();

        if (flOdometer < 0 && frOdometer < 0 && blOdometer < 0)
        {
            telemetry.addData("Direction: ", "Negative and to the left");
            telemetry.update();
        }

        if (flOdometer > 0 && frOdometer > 0 && blOdometer > 0)
        {
            telemetry.addData("Direction: ", "Positive and to the right");
            telemetry.update();
        }
    }
}
