package com.aaravdhawan25.pidautotuner;

/**
 * A single set of PIDF gains, tagged with the tuning rule that produced it
 * so the user can pick between several candidate gain sets after a tune.
 */
public class PIDGains {

    public final String label;
    public final double kP;
    public final double kI;
    public final double kD;
    public final double kF;

    public PIDGains(String label, double kP, double kI, double kD, double kF) {
        this.label = label;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    @Override
    public String toString() {
        return String.format(
                "%-18s  kP=%9.5f  kI=%9.5f  kD=%9.5f  kF=%9.6f",
                label, kP, kI, kD, kF
        );
    }
}
