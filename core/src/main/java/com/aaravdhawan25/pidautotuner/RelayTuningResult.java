package com.aaravdhawan25.pidautotuner;

/**
 * Holds the output of a completed relay-feedback test, including the ultimate
 * gain, ultimate period, and the intermediate measurements from which they
 * were derived.
 *
 * <p>Produced by {@link RelayAutoTuner#computeResult()} and consumed by
 * {@link ZieglerNicholsCalculator#computeCandidates} to generate PID gain
 * candidates.
 */
public class RelayTuningResult {

    /**
     * Ultimate gain {@code Ku}, computed as {@code 4d / (π · a)}, where
     * {@code d} is the relay amplitude and {@code a} is the average
     * half-amplitude of the sustained oscillation.
     */
    public final double Ku;

    /**
     * Ultimate period {@code Tu} in seconds — the average duration of one
     * complete oscillation cycle during the relay test.
     */
    public final double Tu;

    /**
     * Average half-amplitude of the observed oscillation, in the same units
     * as the setpoint (encoder ticks or ticks/sec).
     */
    public final double amplitude;

    /**
     * The relay output magnitude used during the test, retained for reference
     * and reconstruction of {@code Ku}.
     */
    public final double relayAmplitude;

    /**
     * Constructs a relay tuning result.
     *
     * @param Ku             ultimate gain
     * @param Tu             ultimate period in seconds
     * @param amplitude      average half-amplitude of the oscillation
     * @param relayAmplitude relay output magnitude used during the test
     */
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
