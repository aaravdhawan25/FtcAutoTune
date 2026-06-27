package com.aaravdhawan25.pidautotuner;

/**
 * A discrete-time PIDF controller with configurable output bounds and
 * integral anti-windup clamping.
 *
 * <p>This class has no dependency on the FTC SDK and may be unit-tested
 * on a standard JVM. It is intended for use both within the tuner's
 * live-test phase and as a production controller in subsystem code once
 * gains have been determined.
 *
 * <p>The control law evaluated on each call to {@link #calculate} is:
 * <pre>
 *   u(t) = kP · e(t) + kI · ∫e dt + kD · de/dt + kF · target
 * </pre>
 * where {@code e(t) = target − measurement}. The integral accumulator is
 * bounded by {@link #setIntegralSumMax} and the output is clamped to the
 * range set by {@link #setOutputBounds}.
 *
 * <h2>Typical usage</h2>
 * <p>For <b>position control</b> (arms, lifts): set {@code kF = 0} unless
 * a constant gravity-compensation term is required.
 *
 * <p>For <b>velocity control</b> (flywheels, intakes): {@code kF} represents
 * the fraction of full power required to sustain the target speed with zero
 * tracking error, effectively decoupling the feedforward from the feedback path.
 */
public class PIDFController {

    private double kP;
    private double kI;
    private double kD;
    private double kF;

    /** Upper bound on the integral accumulator. Prevents integral windup. */
    private double integralSumMax = Double.POSITIVE_INFINITY;

    /** Lower bound on the controller output. */
    private double outputMin = -1.0;

    /** Upper bound on the controller output. */
    private double outputMax = 1.0;

    private double errorSum = 0.0;
    private double lastError = 0.0;
    private double lastTimestamp = Double.NaN;

    /**
     * Constructs a PIDF controller with the given gains.
     *
     * @param kP proportional gain
     * @param kI integral gain
     * @param kD derivative gain
     * @param kF feedforward gain, multiplied by the target value
     */
    public PIDFController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    /**
     * Convenience factory that constructs a controller from a {@link PIDGains} instance.
     *
     * @param gains the gain set to apply
     * @return a new {@code PIDFController} initialised with the provided gains
     */
    public static PIDFController fromGains(PIDGains gains) {
        return new PIDFController(gains.kP, gains.kI, gains.kD, gains.kF);
    }

    /**
     * Replaces all four gains simultaneously.
     *
     * @param gains the new gain set to apply
     */
    public void setGains(PIDGains gains) {
        this.kP = gains.kP;
        this.kI = gains.kI;
        this.kD = gains.kD;
        this.kF = gains.kF;
    }

    /**
     * Sets the symmetric clamp applied to the integral accumulator.
     * Clamping to a finite value prevents integral windup when the output
     * saturates.
     *
     * @param integralSumMax the maximum absolute value of the integral sum
     */
    public void setIntegralSumMax(double integralSumMax) {
        this.integralSumMax = integralSumMax;
    }

    /**
     * Sets the output saturation limits. The value returned by
     * {@link #calculate} is always clamped to {@code [min, max]}.
     *
     * @param min lower output bound (e.g. {@code -1.0} or {@code 0.0})
     * @param max upper output bound (e.g. {@code 1.0})
     */
    public void setOutputBounds(double min, double max) {
        this.outputMin = min;
        this.outputMax = max;
    }

    /**
     * Clears the integral accumulator and derivative history.
     * Should be called whenever the setpoint changes to avoid transient
     * behaviour caused by stale state from a previous operating point.
     */
    public void reset() {
        errorSum = 0.0;
        lastError = 0.0;
        lastTimestamp = Double.NaN;
    }

    /**
     * Evaluates the PIDF control law and returns the commanded output.
     *
     * @param target           the desired setpoint (position or velocity)
     * @param measurement      the current process variable
     * @param timestampSeconds a monotonically increasing time value in seconds;
     *                         {@code getRuntime()} from an OpMode is appropriate
     * @return the control output, clamped to {@code [outputMin, outputMax]}
     */
    public double calculate(double target, double measurement, double timestampSeconds) {
        double error = target - measurement;

        double dt;
        if (Double.isNaN(lastTimestamp)) {
            dt = 0.0; // First call — no dt available; skip integral and derivative.
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
