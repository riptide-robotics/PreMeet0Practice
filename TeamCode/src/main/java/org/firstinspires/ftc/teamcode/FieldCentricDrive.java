package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Field Centric Drive")
public class FieldCentricDrive extends LinearOpMode {


    // Stating dc motors
    DcMotor flWheel;
    DcMotor frWheel;
    DcMotor blWheel;
    DcMotor brWheel;

    double x;
    double y;
    double rx;

    private int kiy;
    private int kdy;
    private int kpy;

    private int kix;
    private int kdx;
    private int kpx;

    private double previouserry = 0;
    private double previouserrx = 0;



    private double ypos = 0;
    private double xpos = 0;
    private double theta = 0;

    private double yrot = 0;

    private double integraly = 0;
    private double integralx = 0;
    private ElapsedTime time = new ElapsedTime();

    private final double R = 4.8; //
    private final int V = 2000; //
    private final double C = 1/V; // Circumfrence
    private final double L = 10; //unknown I PUT A RANDOM NUMBER (L is the distance between the 2 parallel odomitors


    private IMU imu;



    @Override
    public void runOpMode() throws InterruptedException {

        // reference our motors so program knows what we calling
        flWheel = hardwareMap.dcMotor.get("flWheel");
        frWheel = hardwareMap.dcMotor.get("frWheel");
        blWheel = hardwareMap.dcMotor.get("blWheel");
        brWheel = hardwareMap.dcMotor.get("brWheel");

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters paremeters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

        imu.resetYaw();

        imu.initialize(paremeters);



        int flOdomitorInit = flWheel.getCurrentPosition() ;
        int frOdomitorInit = frWheel.getCurrentPosition();
        int blOdomitorInit = blWheel.getCurrentPosition();

        waitForStart();

        // Reverse direction because gamepad standard is reversed (only for left side)
        flWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        blWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        while (opModeIsActive()) {
            // referencing gamepad stick to varialbe (x multiplied by 1.1 for margin of error
            y = -gamepad1.left_stick_y;
            x = -gamepad1.left_stick_x * 1.1;
            rx = gamepad1.right_stick_x;


            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotY = x * Math.sin(-heading) + y * Math.sin(-heading);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);



            double scale = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            // powers of each motor
            double flPower = (rotY - rotX + rx) / denominator;
            double frPower = (rotY + rotX - rx) / denominator;
            double blPower = (rotY + rotX + rx) / denominator;
            double brPower = (rotY - rotX - rx) / denominator;


            // Set power to each motor
            flWheel.setPower(flPower);
            frWheel.setPower(frPower);
            blWheel.setPower(blPower);
            brWheel.setPower(brPower);

            int flOdomitorlChange = flWheel.getCurrentPosition() - flOdomitorInit; // Front left odomitor
            int frOdomitorChange = frWheel.getCurrentPosition() - frOdomitorInit; // Front right odomitor
            int blOdomitorChange = blWheel.getCurrentPosition() - blOdomitorInit; // Perpendicular odomitor

            double deltaY = blOdomitorChange - (frOdomitorChange - flOdomitorlChange) / 2;
            double deltaX = C*(flOdomitorlChange + frOdomitorChange);
            double deltaTheta = C*(frOdomitorChange - flOdomitorlChange) / L;

            deltaX = deltaX*Math.cos(deltaTheta) - deltaX*Math.sin(deltaTheta);
            deltaY = deltaY*Math.sin(deltaTheta) + deltaY*Math.cos(deltaTheta);


            flOdomitorInit = flWheel.getCurrentPosition();
            frOdomitorInit = frWheel.getCurrentPosition();
            blOdomitorInit = blWheel.getCurrentPosition();



        }
    }
    public void driveYPID(int target, double elapsedTime)
    {
        double erry = ypos - target;
        double integrallimity = 1000;
        elapsedTime = time.seconds();
        integraly += erry * elapsedTime;

        if (Math.abs(integraly) > integrallimity)
        {
            integraly = Math.signum(integraly) * integrallimity;
        }

        double deriviativey = (erry - previouserry) / elapsedTime;
        double outputy = erry * kpy + integraly * kiy + kdy * deriviativey;

        flWheel.setPower(outputy);

        frWheel.setPower(-outputy);
        ;
        brWheel.setPower(outputy);

        blWheel.setPower(-outputy);

        time.reset();
        previouserry = erry;
    }

    public void driveXPID(int target, double elapsedTime){
        double errx = xpos - target;
        elapsedTime = time.milliseconds();
        integralx += errx * elapsedTime;
        int integrallimitx = 1000;

        if (Math.abs(integralx) > integrallimitx){
            integralx = Math.signum(integralx) * integralx;
        }

        double deriviativex = (errx - previouserrx) / elapsedTime;
        double outputx = errx * kpx + integralx * kix + kdx * deriviativex;


        flWheel.setPower(outputx);
        frWheel.setPower(outputx);
        brWheel.setPower(outputx);
        blWheel.setPower(outputx);


        previouserrx = errx;
    }

    public void GoToMethod(int xpos, int ypos)
    {
        ElapsedTime time = new ElapsedTime();
        while(Math.abs(this.xpos - xpos) > 10 && Math.abs(this.ypos - ypos) > 10)
        {

            driveYPID(ypos, time.milliseconds());
            driveXPID(xpos, time.milliseconds());

            time.reset();
        }
    }

    public void rotateXPID(int target)
    {
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double errxrot = rotX - target;
    }
}
