package com.aaravdhawan25.pidautotuner;

import java.util.ArrayList;
import java.util.List;

/**
 * Implements the relay feedback ("bang-bang") auto-tuning method described by
 * Astrom &amp; Hagglund (1984). The output is driven hard in one direction
 * whenever the measurement is below the setpoint (minus a hysteresis band)
 * and hard in the other direction whenever it's above the setpoint (plus the
 * hysteresis band). This forces the system into a sustained, limit-cycle
 * oscillation, from which the ultimate gain {@code Ku} and ultimate period
 * {@code Tu} can be extracted and fed into Ziegler-Nichols style tuning
 * rules (see {@link ZieglerNicholsCalculator}).
 *
 * <p>This class is purely mathematical -- it does not know about motors,
 * encoders, or OpModes. The caller is responsible for, each control-loop
 * iteration:
 * <ol>
 *     <li>taking a measurement (encoder position or velocity)</li>
 *     <li>calling {@link #update(double, double)} with that measurement and
 *         the current timestamp in seconds</li>
 *     <li>applying the returned relay output (e.g. motor.setPower(output))</li>
 *     <li>checking {@link #isFinished()} to know when enough cycles have been
 *         collected</li>
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
     * @param setpoint        the target value to oscillate around (encoder ticks,
     *                         or ticks/sec for velocity tuning)
     * @param relayAmplitude  the magnitude of the relay output, in motor-power
     *                         units, e.g. 0.6 means the relay drives the motor
     *                         to either +0.6 or -0.6. Pick a value large enough
     *                         to overcome static friction but small enough to
     *                         be safe -- 0.3 to 0.7 is typical.
     * @param hysteresis      a small deadband (in the same units as setpoint)
     *                         around the setpoint that the measurement must
     *                         cross before the relay switches. This prevents
     *                         noise from causing chatter. A few encoder ticks
     *                         (position) or a few ticks/sec (velocity) is
     *                         typically enough.
     * @param cyclesToCollect how many full oscillation cycles to average over
     *                         once the system has settled into a steady limit
     *                         cycle. 5-10 is typical.
     * @param cyclesToIgnore  how many initial cycles to discard while the
     *                         system transitions from its starting condition
     *                         into the steady limit cycle. 1-2 is typical.
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
     * Feed in the latest measurement and timestamp, and get back the relay
     * output to apply. Call this once per control-loop iteration.
     *
     * @param measurement       the current measured value (same units as setpoint)
     * @param timestampSeconds  current time in seconds (e.g. from an ElapsedTime)
     * @return the relay output, either {@code +relayAmplitude} or {@code -relayAmplitude}
     */
    public double update(double measurement, double timestampSeconds) {
        // Track the running min/max since the last switch, so that when we
        // do switch, we know how far the system overshot the setpoint.
        if (!hasExtremum) {
            currentExtremum = measurement;
            hasExtremum = true;
        } else if (outputHigh) {
            currentExtremum = Math.max(currentExtremum, measurement);
        } else {
            currentExtremum = Math.min(currentExtremum, measurement);
        }

        // Decide whether to switch the relay.
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
     * @return true once enough oscillation cycles have been recorded to
     *         compute a result. A "cycle" = one high half-period + one low
     *         half-period.
     */
    public boolean isFinished() {
        int usableHalfPeriods = (cyclesToCollect + cyclesToIgnore) * 2;
        return halfPeriods.size() >= usableHalfPeriods;
    }

    /** @return how many full oscillation cycles have been recorded so far. */
    public int cyclesCollected() {
        return halfPeriods.size() / 2;
    }

    /**
     * Computes the ultimate gain {@code Ku} and ultimate period {@code Tu}
     * from the recorded oscillation. Call this once {@link #isFinished()}
     * returns true.
     *
     * @return the result, or {@code null} if not enough data has been collected
     */
    public RelayTuningResult computeResult() {
        int ignoreHalfPeriods = cyclesToIgnore * 2;
        int usableHalfPeriods = cyclesToCollect * 2;

        if (halfPeriods.size() < ignoreHalfPeriods + usableHalfPeriods) {
            return null;
        }

        // Average period: a full cycle = two half-periods (high + low).
        double periodSum = 0;
        int periodCount = 0;
        for (int i = ignoreHalfPeriods; i < ignoreHalfPeriods + usableHalfPeriods; i += 2) {
            periodSum += halfPeriods.get(i) + halfPeriods.get(i + 1);
            periodCount++;
        }
        double Tu = periodSum / periodCount;

        // Average peak-to-peak amplitude. extrema alternate between a high
        // peak and a low trough (or vice versa) -- amplitude 'a' is half the
        // average peak-to-peak swing.
        double amplitudeSum = 0;
        int amplitudeCount = 0;
        int extremaOffset = ignoreHalfPeriods + 1; // extrema[i] corresponds to halfPeriods[i-1]
        for (int i = extremaOffset; i + 1 < extrema.size() && amplitudeCount < usableHalfPeriods / 2; i += 2) {
            double peakToPeak = Math.abs(extrema.get(i) - extrema.get(i + 1));
            amplitudeSum += peakToPeak;
            amplitudeCount++;
        }
        double a = (amplitudeSum / Math.max(amplitudeCount, 1)) / 2.0;

        if (a <= 0) {
            return null;
        }

        // Astrom-Hagglund: Ku = 4*d / (pi*a)
        double Ku = (4.0 * relayAmplitude) / (Math.PI * a);

        return new RelayTuningResult(Ku, Tu, a, relayAmplitude);
    }
}
