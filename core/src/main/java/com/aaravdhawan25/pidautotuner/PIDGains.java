package com.aaravdhawan25.pidautotuner;

/**
 * just holds a set of pid gains with a name
 * nothing fancy, its literally just a data class
 */
public class PIDGains {

    public final String label; // like "PD (classic ZN)" so you know which one it is
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
        // formatted nicely so it looks good on driver station telemetry
        return String.format(
                "%-18s  kP=%9.5f  kI=%9.5f  kD=%9.5f  kF=%9.6f",
                label, kP, kI, kD, kF
        );
    }
}
