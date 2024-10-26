package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class OdometryLocalizer implements Runnable{

    private final DcMotor leftParallel, rightParallel, perpendicular;

    private final int pollRate;

    private final EditablePose2D currPos = new EditablePose2D(0,0,Math.toRadians(90), DistanceUnit.CM);

    private final double ODO_POD_RADIUS = 4.8; //in Cm
    private final int TICKS_PER_REVOLUTION = 2000;
    private final double LATERAL_DISTANCE = DistanceUnit.INCH.toCm(10.9375); // Measured to the best of my ability
    private final double PERP_TO_CENTER = DistanceUnit.INCH.toCm(0.625); // Measured to the best of my ability
    private final double C = 2 * Math.PI * ODO_POD_RADIUS / TICKS_PER_REVOLUTION;

    private final double X_FACTOR = 0.51060306264;
    private final double Y_FACTOR = 1;
    private final double ANGLE_FACTOR = 1;

    private int leftParallelTicks;
    private int rightParallelTicks;
    private int perpendicularTicks;

    private int previousLeftParallelTicks = 0;
    private int previousRightParallelTicks = 0;
    private int previousPerpendicularTicks = 0;

    private boolean running = true;

    public void stop(){
        running = false;
    }


    public OdometryLocalizer(DcMotor leftParallel, DcMotor rightParallel, DcMotor perpendicular, int pollRate){
        this.leftParallel = leftParallel;
        this.rightParallel = rightParallel;
        this.perpendicular = perpendicular;
        this.pollRate = pollRate;
    }

    public void threeWheelLocalize() {
        leftParallelTicks = -leftParallel.getCurrentPosition();
        rightParallelTicks = -rightParallel.getCurrentPosition();
        perpendicularTicks = -perpendicular.getCurrentPosition();

        int dodo1 = leftParallelTicks - previousLeftParallelTicks;
        int dodo2 = rightParallelTicks - previousRightParallelTicks;
        int dodo3 = perpendicularTicks - previousPerpendicularTicks;

        double dx = (C * (dodo1 + dodo2)) / 2 * X_FACTOR;
        double dy = C * (dodo3 - (PERP_TO_CENTER * (dodo1 - dodo2)) / LATERAL_DISTANCE) * Y_FACTOR;
        double dtheta = (C * (dodo1 - dodo2)) / LATERAL_DISTANCE * ANGLE_FACTOR;

        dx = dx * Math.cos(currPos.getH()) - dx * Math.sin(currPos.getH());
        dy = dy * Math.sin(currPos.getH()) + dy * Math.cos(currPos.getH());

        previousLeftParallelTicks = leftParallelTicks;
        previousRightParallelTicks = rightParallelTicks;
        previousPerpendicularTicks = perpendicularTicks;

        currPos.setX(currPos.getX(DistanceUnit.CM) + dx, DistanceUnit.CM);
        currPos.setY(currPos.getY(DistanceUnit.CM) + dy, DistanceUnit.CM);
        currPos.setH(currPos.getH() + dtheta);

        double thetaNormalized = Math.atan2(Math.sin(currPos.getH()), Math.cos(currPos.getH()));
        currPos.setH(thetaNormalized);
    }

    public EditablePose2D getCurrPos(){
        return currPos;
    }
    public int getLeftEncoder() {return leftParallelTicks;}
    public int getRightEncoder() {return rightParallelTicks;}
    public int getPerpendicularEncoder() {return perpendicularTicks;}

    @Override
    public void run(){
        while(running){
            threeWheelLocalize();
            try {
                Thread.sleep(pollRate);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }
}
