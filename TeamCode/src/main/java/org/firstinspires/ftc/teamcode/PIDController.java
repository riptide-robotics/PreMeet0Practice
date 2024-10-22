package org.firstinspires.ftc.teamcode;

public class PIDController {

    private double kP, kI, kD;  // PID gains
    private double integralSum = 0;  // Sum of errors (for integral term)
    private double lastError = 0;  // Last error (for derivative term)
    private double lastTime = 0;  // Time of last update

    // Constructor to set the gains
    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    // Method to reset the controller (e.g., when starting a new movement)
    public void reset() {
        integralSum = 0;
        lastError = 0;
        lastTime = System.currentTimeMillis();  // Reset the time reference
    }

    // The main PID calculation method
    public double calculate(double error) {
        // Get the current time and calculate the time difference
        double currentTime = System.currentTimeMillis();
        double dt = (currentTime - lastTime) / 1000.0;  // Convert milliseconds to seconds

        if (dt <= 0) {
            dt = 0.01;  // Avoid division by zero or very small values
        }

        // Proportional term
        double proportional = kP * error;

        // Integral term (sum of errors over time)
        integralSum += error * dt;
        double integral = kI * integralSum;

        // Derivative term (rate of change of error)
        double derivative = kD * (error - lastError) / dt;

        // Store current error and time for the next calculation
        lastError = error;
        lastTime = currentTime;

        // Return the sum of the three PID terms as the output
        return proportional + integral + derivative;
    }

    // Method to tune the PID gains
    public void setPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
}

