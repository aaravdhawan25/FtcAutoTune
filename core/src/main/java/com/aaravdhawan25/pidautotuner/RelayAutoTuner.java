package com.aaravdhawan25.pidautotuner;

import java.util.ArrayList;
import java.util.List;

/**
 * Implements the relay feedback auto-tuning method (Åström & Hägglund, 1984).
 *
 * <p>The output alternates between {@code +relayAmplitude} and {@code -relayAmplitude}
 * based on whether the measurement is above or below the setpoint. This forces
 * the system into a sustained limit-cycle oscillation, from which the ultimate
 * gain {@code Ku} and ultimate period {@code Tu} are extracted and passed to
 * {@link ZieglerNicholsCalculator} to produce PID gain candidates.
 *
 * <p>This class is purely mathematical and has no dependency on the FTC SDK.
 * Each loop iteration, the caller must:
 * <ol>
 *   <li>Read the encoder (position or velocity).</li>
 *   <li>Call {@link #update(double, double)} with that measurement and the
 *       current timestamp.</li>
 *   <li>Apply the returned value to the motor via {@code motor.setPower(...)}.</li>
 *   <li>Check {@link #isFinished()} and call {@link #computeResult()} when true.</li>
 * </ol>
 */
public class RelayAutoTuner {

    private final double setpoint;
    private final double relayAmplitude;
    private final double hysteresis;
    private final int cyclesToCollect;
    private final int cyclesToIgnore;

    private boolean outputHigh = true;
    private double currentExtremum;
    private boolean hasExtremum = false;

    private Double lastSwitchTime = null;

    private final List<Double> halfPeriods = new ArrayList<>();
    private final List<Double> extrema = new ArrayList<>();

    /**
     * Constructs a new {@code RelayAutoTuner}.
     *
     * @param setpoint        the target value to oscillate around, in encoder
     *                        ticks for position or ticks/sec for velocity
     * @param relayAmplitude  the relay output magnitude in motor power units
     *                        (0 to 1). Must be large enough to overcome static
     *                        friction but small enough to be mechanically safe.
     *                        Typical range: 0.3 to 0.7.
     * @param hysteresis      a deadband around the setpoint that prevents
     *                        sensor noise from causing rapid relay switching.
     *                        A few encoder ticks or a few ticks/sec is sufficient.
     * @param cyclesToCollect the number of full oscillation cycles to average
     *                        when computing {@code Ku} and {@code Tu}. More
     *                        cycles improve accuracy at the cost of tuning time.
     *                        Five to ten is typical.
     * @param cyclesToIgnore  the number of initial cycles to discard while the
     *                        system transitions into a steady limit cycle.
     *                        One to two is typical.
     */
    public RelayAutoTuner(double setpoint, double relayAmplitude, double hysteresis,
                           int cyclesToCollect, int cyclesToIgnore) {
        if (relayAmplitude <= 0) {
            throw new IllegalArgumentException("relayAmplitude must be positive");
        }
        if (hysteresis < 0) {
            throw new IllegalArgumentException("hysteresis must be non-negative");
        }
        this.setpoint = setpoint;
        this.relayAmplitude = relayAmplitude;
        this.hysteresis = hysteresis;
        this.cyclesToCollect = cyclesToCollect;
        this.cyclesToIgnore = cyclesToIgnore;
    }

    /**
     * Advances the relay test by one control-loop iteration.
     *
     * @param measurement      the current sensor reading, in the same units
     *                         as the setpoint
     * @param timestampSeconds current time in seconds (e.g. {@code getRuntime()})
     * @return the relay output to apply to the motor, either
     *         {@code +relayAmplitude} or {@code -relayAmplitude}
     */
    public double update(double measurement, double timestampSeconds) {
        // Track the peak or trough since the last relay switch so that the
        // oscillation amplitude can be computed when computeResult() is called.
        if (!hasExtremum) {
            currentExtremum = measurement;
            hasExtremum = true;
        } else if (outputHigh) {
            currentExtremum = Math.max(currentExtremum, measurement);
        } else {
            currentExtremum = Math.min(currentExtremum, measurement);
        }

        // Switch the relay when the measurement crosses the hysteresis threshold.
        if (outputHigh && measurement > setpoint + hysteresis) {
            recordSwitch(timestampSeconds);
            outputHigh = false;
            hasExtremum = false;
        } else if (!outputHigh && measurement < setpoint - hysteresis) {
            recordSwitch(timestampSeconds);
            outputHigh = true;
            hasExtremum = false;
        }

        return outputHigh ? relayAmplitude : -relayAmplitude;
    }

    private void recordSwitch(double timestampSeconds) {
        extrema.add(currentExtremum);
        if (lastSwitchTime != null) {
            halfPeriods.add(timestampSeconds - lastSwitchTime);
        }
        lastSwitchTime = timestampSeconds;
    }

    /**
     * Returns {@code true} when enough oscillation cycles have been recorded
     * to compute a valid result. One cycle consists of one high half-period
     * followed by one low half-period.
     */
    public boolean isFinished() {
        int usableHalfPeriods = (cyclesToCollect + cyclesToIgnore) * 2;
        return halfPeriods.size() >= usableHalfPeriods;
    }

    /**
     * Returns the number of complete oscillation cycles recorded so far.
     * This value increases until {@link #isFinished()} returns {@code true}.
     */
    public int cyclesCollected() {
        return halfPeriods.size() / 2;
    }

    /**
     * Computes the ultimate gain {@code Ku} and ultimate period {@code Tu}
     * from the recorded oscillation data. Must be called after
     * {@link #isFinished()} returns {@code true}.
     *
     * <p>Uses the Åström–Hägglund formula:
     * {@code Ku = 4d / (π · a)}, where {@code d} is the relay amplitude and
     * {@code a} is the average half-amplitude of the sustained oscillation.
     *
     * @return the tuning result, or {@code null} if insufficient data exists
     */
    public RelayTuningResult computeResult() {
        int ignoreHalfPeriods = cyclesToIgnore * 2;
        int usableHalfPeriods = cyclesToCollect * 2;

        if (halfPeriods.size() < ignoreHalfPeriods + usableHalfPeriods) {
            return null;
        }

        // Average the period across the usable cycles (each full cycle = two half-periods).
        double periodSum = 0;
        int periodCount = 0;
        for (int i = ignoreHalfPeriods; i < ignoreHalfPeriods + usableHalfPeriods; i += 2) {
            periodSum += halfPeriods.get(i) + halfPeriods.get(i + 1);
            periodCount++;
        }
        double Tu = periodSum / periodCount;

        // Compute the average half-amplitude from the recorded extrema.
        // Consecutive extrema alternate between peaks and troughs, so the
        // peak-to-trough distance divided by two gives the half-amplitude.
        double amplitudeSum = 0;
        int amplitudeCount = 0;
        int extremaOffset = ignoreHalfPeriods + 1;
        for (int i = extremaOffset; i + 1 < extrema.size() && amplitudeCount < usableHalfPeriods / 2; i += 2) {
            double peakToPeak = Math.abs(extrema.get(i) - extrema.get(i + 1));
            amplitudeSum += peakToPeak;
            amplitudeCount++;
        }
        double a = (amplitudeSum / Math.max(amplitudeCount, 1)) / 2.0;

        if (a <= 0) {
            return null;
        }

        // Åström–Hägglund ultimate gain formula.
        double Ku = (4.0 * relayAmplitude) / (Math.PI * a);

        return new RelayTuningResult(Ku, Tu, a, relayAmplitude);
    }
}
