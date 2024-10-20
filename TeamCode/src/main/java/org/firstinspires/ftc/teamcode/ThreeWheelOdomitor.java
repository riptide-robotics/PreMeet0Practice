package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Mat;

public class ThreeWheelOdomitor{

    DcMotor flWheel;
    DcMotor frWheel;
    DcMotor brWheel;
    DcMotor blWheel;





    private final double R = 4.8;
    private final int V = 2000;
    private final double C = 1/V;
    private final double L = 10; //random number havent measured yet

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


    public void odomitor(HardwareMap hardwareMap, Gamepad gamepad1) {

        flWheel = hardwareMap.dcMotor.get("flWheel");
        frWheel = hardwareMap.dcMotor.get("frWheel");
        blWheel = hardwareMap.dcMotor.get("blWheel");
        brWheel = hardwareMap.dcMotor.get("brWheel");


        int flOdomitorInit = flWheel.getCurrentPosition();
        int frOdomitorInit = flWheel.getCurrentPosition();
        int blOdomitorInit = blWheel.getCurrentPosition();


        int flOdomitorChange = flWheel.getCurrentPosition() - flOdomitorInit;
        int frOdomitorChange = frWheel.getCurrentPosition() - frOdomitorInit;
        int blOdomitorChange = blWheel.getCurrentPosition() - blOdomitorInit;


        double dx = C * (flOdomitorChange + frOdomitorChange);
        double dy = blOdomitorChange - (frOdomitorChange - flOdomitorChange) / 2;
        double deltaTheta = C * (frOdomitorChange - flOdomitorChange) / L;

        dx = dx * Math.cos(theta) - dx * Math.sin(theta);
        dy = dy * Math.cos(theta) - dy * Math.sin(theta);

        xpos += dx;
        ypos += dy;
        theta += deltaTheta;


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

    public void rotateYPID()
    {

    }
}