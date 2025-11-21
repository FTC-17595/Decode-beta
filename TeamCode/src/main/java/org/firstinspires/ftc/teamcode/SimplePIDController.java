package org.firstinspires.ftc.teamcode;

/**
 * Lightweight PID controller for teleop alignment tasks.
 * Calculates control effort from the provided error (setpoint - measurement or otherwise).
 */
public class SimplePIDController {

    private double kP;
    private double kI;
    private double kD;

    private double integralSum;
    private double previousError;
    private boolean firstUpdate = true;

    private double integralMin = Double.NEGATIVE_INFINITY;
    private double integralMax = Double.POSITIVE_INFINITY;

    public SimplePIDController(double kP, double kI, double kD) {
        setCoefficients(kP, kI, kD);
    }

    public void setCoefficients(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setIntegralBounds(double integralMin, double integralMax) {
        this.integralMin = integralMin;
        this.integralMax = integralMax;
    }

    public double calculate(double error, double dtSeconds) {
        if (dtSeconds <= 0) {
            dtSeconds = 1e-3;
        }

        integralSum += error * dtSeconds;
        integralSum = Math.max(integralMin, Math.min(integralSum, integralMax));

        double derivative = 0.0;
        if (!firstUpdate) {
            derivative = (error - previousError) / dtSeconds;
        } else {
            firstUpdate = false;
        }

        previousError = error;
        return (kP * error) + (kI * integralSum) + (kD * derivative);
    }

    public void reset() {
        integralSum = 0.0;
        previousError = 0.0;
        firstUpdate = true;
    }

    public double getIntegralSum() {
        return integralSum;
    }
}


