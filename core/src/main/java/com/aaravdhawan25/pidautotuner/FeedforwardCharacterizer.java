package com.aaravdhawan25.pidautotuner;

import java.util.ArrayList;
import java.util.List;

/**
 * Computes the feedforward gain {@code kF} for velocity control.
 *
 * <p>Relay feedback (see {@link RelayAutoTuner}) measures the dynamics of the
 * system but does not by itself tell you how much power is needed to sustain
 * a given velocity with zero error -- that's the feedforward term. This class
 * characterizes it separately: drive the motor open-loop at one or more fixed
 * power levels, let the velocity settle, and record (power, steady-state
 * velocity) pairs. {@code kF} is then the slope of the best-fit line through
 * the origin: {@code power = kF * velocity}.
 *
 * <p>For a single test point (the simplest case -- run the motor at full
 * power and record the steady-state velocity), this reduces to the familiar
 * {@code kF = 1.0 / maxVelocity} (in power-per-tick/sec units).
 */
public class FeedforwardCharacterizer {

    private final List<double[]> samples = new ArrayList<>();

    /**
     * Record one (power, steady-state velocity) sample. Call this once per
     * power level tested, after the velocity has settled (e.g. after running
     * at that power for ~1-2 seconds).
     *
     * @param power              open-loop motor power applied, e.g. 0.5, 0.75, 1.0
     * @param steadyStateVelocity the resulting steady-state velocity, in the
     *                            same units (ticks/sec) the controller's
     *                            target velocity will be specified in
     */
    public void addSample(double power, double steadyStateVelocity) {
        if (steadyStateVelocity == 0) {
            return; // avoid div-by-zero / degenerate samples
        }
        samples.add(new double[]{power, steadyStateVelocity});
    }

    public int sampleCount() {
        return samples.size();
    }

    /**
     * Computes kF as the least-squares slope of {@code power = kF * velocity}
     * through the origin.
     *
     * @return kF, or {@code Double.NaN} if no valid samples have been recorded
     */
    public double computeKf() {
        if (samples.isEmpty()) {
            return Double.NaN;
        }

        double numerator = 0;
        double denominator = 0;
        for (double[] sample : samples) {
            double power = sample[0];
            double velocity = sample[1];
            numerator += power * velocity;
            denominator += velocity * velocity;
        }

        if (denominator == 0) {
            return Double.NaN;
        }

        return numerator / denominator;
    }
}
