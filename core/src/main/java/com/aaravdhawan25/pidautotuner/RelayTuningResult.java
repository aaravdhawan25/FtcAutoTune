package com.aaravdhawan25.pidautotuner;

/**
 * Raw output of a relay-feedback test: the ultimate gain and period,
 * plus the measurements used to compute them.
 */
public class RelayTuningResult {

    /** Ultimate gain, computed as {@code 4*d / (pi*a)}. */
    public final double Ku;

    /** Ultimate period, in seconds -- the period of the sustained oscillation. */
    public final double Tu;

    /** Oscillation amplitude (half the average peak-to-peak swing), in setpoint units. */
    public final double amplitude;

    /** The relay output amplitude {@code d} that was used to produce the oscillation. */
    public final double relayAmplitude;

    public RelayTuningResult(double Ku, double Tu, double amplitude, double relayAmplitude) {
        this.Ku = Ku;
        this.Tu = Tu;
        this.amplitude = amplitude;
        this.relayAmplitude = relayAmplitude;
    }

    @Override
    public String toString() {
        return String.format(
                "Ku=%.5f  Tu=%.4fs  amplitude=%.3f  relayAmplitude=%.3f",
                Ku, Tu, amplitude, relayAmplitude
        );
    }
}
