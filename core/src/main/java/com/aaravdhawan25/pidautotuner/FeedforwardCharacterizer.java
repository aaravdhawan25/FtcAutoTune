package com.aaravdhawan25.pidautotuner;

import java.util.ArrayList;
import java.util.List;

/**
 * figures out kF (feedforward gain) for velocity control
 *
 * so the relay test tells us how the system responds to changes but
 * it doesnt tell us how much power we need to just hold a steady speed.
 * thats what kF is for - its basically "how much power does this motor
 * need at this speed to stay there without the PID having to fight it"
 *
 * how it works:
 * spin the motor at a few different fixed power levels (like 0.5, 0.75, 1.0)
 * wait for it to settle
 * record (power, velocity) pairs
 * fit a line through the origin: power = kF * velocity
 * slope of that line = kF
 *
 * the math is just least squares regression but through the origin
 * which simplifies to kF = sum(p*v) / sum(v*v)
 */
public class FeedforwardCharacterizer {

    private final List<double[]> samples = new ArrayList<>();

    /**
     * add a data point - call this once per power level after the
     * motor has settled (wait like 1-2 seconds at each power level)
     *
     * @param power               the motor power you tested (0.5, 0.75, 1.0 etc)
     * @param steadyStateVelocity how fast it was going after it settled (ticks/sec)
     */
    public void addSample(double power, double steadyStateVelocity) {
        if (steadyStateVelocity == 0) {
            return; // skip this, would cause divide by zero issues later
        }
        samples.add(new double[]{power, steadyStateVelocity});
    }

    public int sampleCount() {
        return samples.size();
    }

    /**
     * computes kF using least squares through the origin
     * basically: kF = sum(power * velocity) / sum(velocity^2)
     *
     * @return kF, or NaN if we have no samples yet
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
