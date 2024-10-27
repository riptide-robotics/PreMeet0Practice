package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class pikachu7911PIDAttemptOne {
    private static double kP = 0;
    private static double kI = 0;
    private static double kD = 0;

    public double integral = 0;
    public double previousError = 0;

    DcMotor lLiftSlide;
    DcMotor rLiftSlide;

    final int COUNTS_PER_REV = 752;
    public static int highestHigh = 2350;
    public static int lowestLow = 800;

    ElapsedTime time = new ElapsedTime();

    public pikachu7911PIDAttemptOne(HardwareMap hardwareMap) {
        lLiftSlide = hardwareMap.dcMotor.get("lSlide");
        rLiftSlide = hardwareMap.dcMotor.get("rSlide");
    }

    public void slidePID(int goalPos) {
        int currPos = rLiftSlide.getCurrentPosition();

        double elapsedTime = time.seconds();
        double currError = goalPos - currPos;

        integral += currError * elapsedTime;


    }
}
