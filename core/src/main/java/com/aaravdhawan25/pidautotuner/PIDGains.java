package com.aaravdhawan25.pidautotuner;

/**
 * An immutable value object holding a complete set of PIDF gains together
 * with a descriptive label identifying the tuning rule that produced them.
 *
 * <p>Instances are produced by {@link ZieglerNicholsCalculator} and
 * presented to the user as a ranked list of candidates on Driver Station
 * telemetry following a successful relay test.
 */
public class PIDGains {

    /** Human-readable label identifying the tuning rule (e.g. "PD (classic ZN)"). */
    public final String label;

    public final double kP;
    public final double kI;
    public final double kD;
    public final double kF;

    /**
     * Constructs a gain set with the specified values and label.
     *
     * @param label descriptive name of the tuning rule
     * @param kP    proportional gain
     * @param kI    integral gain
     * @param kD    derivative gain
     * @param kF    feedforward gain
     */
    public PIDGains(String label, double kP, double kI, double kD, double kF) {
        this.label = label;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    /**
     * Returns a fixed-width formatted string suitable for Driver Station
     * telemetry display.
     */
    @Override
    public String toString() {
        return String.format(
                "%-18s  kP=%9.5f  kI=%9.5f  kD=%9.5f  kF=%9.6f",
                label, kP, kI, kD, kF
        );
    }
}
