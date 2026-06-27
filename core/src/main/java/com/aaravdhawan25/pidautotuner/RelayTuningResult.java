package com.aaravdhawan25.pidautotuner;

/**
 * holds the output of the relay test
 * just Ku and Tu basically plus some extra info
 *
 * Ku = ultimate gain (how aggressive you can go before it goes unstable)
 * Tu = ultimate period (how long one full oscillation takes in seconds)
 *
 * these get fed into ZieglerNicholsCalculator to get actual kP/kI/kD
 */
public class RelayTuningResult {

    /** ultimate gain - computed as 4*relayAmplitude / (pi * oscillationAmplitude) */
    public final double Ku;

    /** ultimate period in seconds - basically how long each full oscillation took on average */
    public final double Tu;

    /** half the average peak-to-peak swing of the oscillation */
    public final double amplitude;

    /** the relay power we used during the test (just stored for reference) */
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
