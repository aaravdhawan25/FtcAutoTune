package com.aaravdhawan25.pidautotuner;

import java.util.ArrayList;
import java.util.List;

/**
 * Identifies the feedforward gain {@code kF} for a velocity-controlled mechanism
 * via open-loop characterisation.
 *
 * <p>Relay feedback identifies closed-loop dynamics but does not quantify the
 * power required to sustain a given velocity at zero tracking error. This class
 * addresses that by recording (power, steady-state velocity) pairs obtained by
 * running the motor open-loop at several fixed power levels. The feedforward
 * gain is then computed as the least-squares slope of the line
 * {@code power = kF · velocity} constrained to pass through the origin:
 *
 * <pre>
 *   kF = Σ(power · velocity) / Σ(velocity²)
 * </pre>
 *
 * <p>For a single test point at full power this reduces to the familiar
 * {@code kF = 1 / maxVelocity}.
 */
public class FeedforwardCharacterizer {

    private final List<double[]> samples = new ArrayList<>();

    /**
     * Records one characterisation sample. Call this once per power level
     * after the motor velocity has settled (typically 1–2 seconds at each level).
     *
     * @param power               the open-loop power applied (e.g. 0.5, 0.75, 1.0)
     * @param steadyStateVelocity the resulting steady-state velocity in ticks/sec;
     *                            zero-velocity samples are silently discarded to
     *                            avoid numerical singularities
     */
    public void addSample(double power, double steadyStateVelocity) {
        if (steadyStateVelocity == 0) {
            return;
        }
        samples.add(new double[]{power, steadyStateVelocity});
    }

    /**
     * Returns the number of valid samples currently held.
     */
    public int sampleCount() {
        return samples.size();
    }

    /**
     * Computes the feedforward gain {@code kF} using ordinary least squares
     * regression through the origin across all recorded samples.
     *
     * @return the estimated {@code kF}, or {@link Double#NaN} if no valid
     *         samples have been added
     */
    public double computeKf() {
        if (samples.isEmpty()) {
            return Double.NaN;
        }

        double numerator = 0;
        double denominator = 0;
        for (double[] sample : samples) {
            double power    = sample[0];
            double velocity = sample[1];
            numerator   += power * velocity;
            denominator += velocity * velocity;
        }

        if (denominator == 0) {
            return Double.NaN;
        }

        return numerator / denominator;
    }
}
