package com.aaravdhawan25.pidautotuner;

/**
 * a basic PIDF controller that you can use in your actual robot code
 * after you get the gains from the tuner
 *
 * works for both position control (arms, lifts) and velocity control
 * (flywheels, intakes)
 *
 * for position: set kF to 0 unless you need gravity compensation
 * for velocity: kF is how much power it takes to hold the target speed
 *               basically feedforward so the P term doesnt have to do
 *               all the work
 *
 * this class has no FTC SDK stuff in it so you can test it on a laptop
 * if you want to (its just math)
 */
public class PIDFController {

    private double kP;
    private double kI;
    private double kD;
    private double kF;

    /** clamps the integral so it doesnt go crazy (windup is really bad) */
    private double integralSumMax = Double.POSITIVE_INFINITY;

    /** clamps the final output, motor power has to be between -1 and 1 */
    private double outputMin = -1.0;
    private double outputMax = 1.0;

    private double errorSum = 0.0;
    private double lastError = 0.0;
    private double lastTimestamp = Double.NaN;

    public PIDFController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public static PIDFController fromGains(PIDGains gains) {
        return new PIDFController(gains.kP, gains.kI, gains.kD, gains.kF);
    }

    public void setGains(PIDGains gains) {
        this.kP = gains.kP;
        this.kI = gains.kI;
        this.kD = gains.kD;
        this.kF = gains.kF;
    }

    public void setIntegralSumMax(double integralSumMax) {
        this.integralSumMax = integralSumMax;
    }

    public void setOutputBounds(double min, double max) {
        this.outputMin = min;
        this.outputMax = max;
    }

    /**
     * call this when the target changes to clear out old integral/derivative
     * state. if you dont call this the controller might act weird when you
     * switch targets
     */
    public void reset() {
        errorSum = 0.0;
        lastError = 0.0;
        lastTimestamp = Double.NaN;
    }

    /**
     * the main function, call this every loop
     *
     * @param target           where you want to be
     * @param measurement      where you actually are
     * @param timestampSeconds current time in seconds (getRuntime() works)
     * @return motor power to apply, already clamped to [outputMin, outputMax]
     */
    public double calculate(double target, double measurement, double timestampSeconds) {
        double error = target - measurement;

        double dt;
        if (Double.isNaN(lastTimestamp)) {
            // first call so there's no dt yet
            dt = 0.0;
        } else {
            dt = timestampSeconds - lastTimestamp;
            if (dt <= 0) dt = 0.0; // just in case time goes backward somehow idk
        }

        if (dt > 0) {
            errorSum += error * dt;
            errorSum = clamp(errorSum, -integralSumMax, integralSumMax);
        }

        // derivative = how fast the error is changing
        // if dt is 0 we just skip it so we dont divide by zero
        double derivative = (dt > 0) ? (error - lastError) / dt : 0.0;

        double output = kP * error + kI * errorSum + kD * derivative + kF * target;

        lastError = error;
        lastTimestamp = timestampSeconds;

        return clamp(output, outputMin, outputMax);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public double getKP() { return kP; }
    public double getKI() { return kI; }
    public double getKD() { return kD; }
    public double getKF() { return kF; }
}
