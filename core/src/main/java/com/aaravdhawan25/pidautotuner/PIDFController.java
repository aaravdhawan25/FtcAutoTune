package com.aaravdhawan25.pidautotuner;

/**
 * A generic PIDF controller, decoupled from any FTC SDK classes so it can be
 * unit tested on a desktop JVM and reused both by the tuner OpModes and by
 * the user's final subsystem code.
 *
 * <p>Two typical use cases:
 * <ul>
 *     <li><b>Position control</b> (e.g. run-to-position arm/lift): leave
 *     {@code kF} at 0, or set it to a constant gravity-feedforward term.</li>
 *     <li><b>Velocity control</b> (e.g. flywheel): {@code kF} is multiplied
 *         by the target velocity, representing "how much power is needed to
 *         sustain this speed with zero error".</li>
 * </ul>
 */
public class PIDFController {

    private double kP;
    private double kI;
    private double kD;
    private double kF;

    /** Optional clamp applied to the integral accumulator to prevent windup. */
    private double integralSumMax = Double.POSITIVE_INFINITY;

    /** Optional clamp applied to the final output (e.g. motor power is [-1, 1]). */
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

    /** Resets the integral accumulator and derivative history. Call this whenever the target changes. */
    public void reset() {
        errorSum = 0.0;
        lastError = 0.0;
        lastTimestamp = Double.NaN;
    }

    /**
     * Computes the controller output.
     *
     * @param target          desired position or velocity
     * @param measurement     current measured position or velocity
     * @param timestampSeconds monotonically increasing timestamp, in seconds
     * @return the control output, clamped to [outputMin, outputMax]
     */
    public double calculate(double target, double measurement, double timestampSeconds) {
        double error = target - measurement;

        double dt;
        if (Double.isNaN(lastTimestamp)) {
            dt = 0.0;
        } else {
            dt = timestampSeconds - lastTimestamp;
            if (dt <= 0) dt = 0.0;
        }

        if (dt > 0) {
            errorSum += error * dt;
            errorSum = clamp(errorSum, -integralSumMax, integralSumMax);
        }

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
