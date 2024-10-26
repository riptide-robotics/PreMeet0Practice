package org.firstinspires.ftc.teamcode.Autonomous.Pathing;

public class TrapezoidalMotionProfile {

    private double maxA;
    private double maxV;

    //woah look at this cool documentation
    private double dta;               // Time to reach max velocity or peak velocity
    private double mV;                // Effective max velocity (can be lower than maxV if distance is short)
    private double accDist;           // Distance covered during acceleration
    private double cruiseDist;        // Distance covered during cruising
    private double cruiseTime;        // Time spent cruising
    private double decelerationTime;  // Start time of deceleration phase
    private double totalTime;         // Total time for the entire motion profile

    public TrapezoidalMotionProfile(double maxA, double maxV) {
        this.maxA = maxA;
        this.maxV = maxV;
    }

    // Method to calculate the motion profile and store values
    public void calculateProfile(double distance) {
        dta = maxV / maxA;
        double halfDistance = distance / 2;
        accDist = 0.5 * maxA * Math.pow(dta, 2);

        mV = maxV;
        if (accDist > halfDistance) {
            dta = Math.sqrt(2 * halfDistance / maxA);
            mV = dta * maxA;
            accDist = 0.5 * maxA * Math.pow(dta, 2);
        }

        cruiseDist = distance - 2 * accDist;
        cruiseTime = cruiseDist / mV;
        decelerationTime = dta + cruiseTime;
        totalTime = dta + cruiseTime + dta;
    }

    // Method to get the expected position at a specific time
    public double getExpectedPosition(double time) {
        // If time exceeds the total duration, return the target distance
        if (time > totalTime) {
            return accDist + cruiseDist + accDist; // This is the full distance
        }

        // Determine position based on the current phase of the profile
        if (time < dta) { // Acceleration phase
            return 0.5 * maxA * Math.pow(time, 2);
        } else if (time < decelerationTime) { // Cruise phase
            double currCruiseTime = time - dta;
            return accDist + mV * currCruiseTime;
        } else { // Deceleration phase
            double decelTimeElapsed = time - decelerationTime;
            return accDist + cruiseDist + mV * decelTimeElapsed - 0.5 * maxA * Math.pow(decelTimeElapsed, 2);
        }
    }
}
