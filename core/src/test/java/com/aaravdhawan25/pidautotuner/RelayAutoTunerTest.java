package com.aaravdhawan25.pidautotuner;

import org.junit.jupiter.api.Test;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * These tests don't talk to any real hardware -- they simulate a simple
 * first-order-plus-delay "plant" (a stand-in for a motor + encoder) and run
 * it through {@link RelayAutoTuner} to confirm the algorithm produces a
 * sensible Ku/Tu, and that {@link ZieglerNicholsCalculator} converts those
 * into plausible gain sets.
 */
class RelayAutoTunerTest {

    /**
     * Simulates dv/dt = (gain*power - v) / tau, with a small pure transport
     * delay applied to the power signal (typical of real motor/encoder
     * loops, and what gives a first-order system a finite-frequency limit
     * cycle under relay feedback).
     */
    private static class FirstOrderPlusDelayPlant {
        final double gain;
        final double tau;
        final double dt;
        final Deque<Double> delayBuffer = new ArrayDeque<>();
        double value = 0;

        FirstOrderPlusDelayPlant(double gain, double tau, double dt, int delaySamples) {
            this.gain = gain;
            this.tau = tau;
            this.dt = dt;
            for (int i = 0; i < delaySamples; i++) {
                delayBuffer.add(0.0);
            }
        }

        double step(double power) {
            delayBuffer.addLast(power);
            double delayedPower = delayBuffer.removeFirst();
            double dvdt = (gain * delayedPower - value) / tau;
            value += dvdt * dt;
            return value;
        }
    }

    @Test
    void relayTunerFindsOscillationOnSimulatedPlant() {
        double dt = 0.02; // 50 Hz control loop, typical for FTC
        FirstOrderPlusDelayPlant plant = new FirstOrderPlusDelayPlant(
                /* gain */ 2000, /* tau */ 0.3, dt, /* delaySamples */ 3
        );

        double setpoint = 1000;
        RelayAutoTuner tuner = new RelayAutoTuner(
                setpoint,
                /* relayAmplitude */ 0.6,
                /* hysteresis */ 5,
                /* cyclesToCollect */ 5,
                /* cyclesToIgnore */ 2
        );

        double t = 0;
        int maxIterations = 200_000;
        int iterations = 0;

        while (!tuner.isFinished() && iterations < maxIterations) {
            double output = tuner.update(plant.value, t);
            plant.step(output);
            t += dt;
            iterations++;
        }

        assertTrue(tuner.isFinished(), "Tuner should finish within the iteration budget");

        RelayTuningResult result = tuner.computeResult();
        assertNotNull(result, "computeResult() should not be null once finished");

        assertTrue(result.Ku > 0, "Ku should be positive, was " + result.Ku);
        assertTrue(result.Tu > 0, "Tu should be positive, was " + result.Tu);
        // With a 3-sample delay at dt=0.02s, the limit cycle period should be
        // on the order of tens of milliseconds to a couple seconds.
        assertTrue(result.Tu < 5.0, "Tu seems unreasonably large: " + result.Tu);
    }

    @Test
    void ziegerNicholsProducesOrderedConservativeToAggressiveGains() {
        RelayTuningResult result = new RelayTuningResult(/* Ku */ 1.0, /* Tu */ 0.5, /* amplitude */ 50, /* relayAmplitude */ 0.6);

        List<PIDGains> candidates = ZieglerNicholsCalculator.computeCandidates(result, 0.001);

        assertEquals(6, candidates.size());

        // P-only candidate should have zero I and D.
        PIDGains pOnly = candidates.get(0);
        assertEquals(0.5 * result.Ku, pOnly.kP, 1e-9);
        assertEquals(0, pOnly.kI, 1e-9);
        assertEquals(0, pOnly.kD, 1e-9);

        // Pessen (most aggressive) should have the highest kP.
        PIDGains pessen = candidates.get(candidates.size() - 1);
        for (PIDGains gains : candidates) {
            assertTrue(pessen.kP >= gains.kP, "Pessen kP should be >= " + gains.label);
        }

        // kF should be passed through unchanged to every candidate.
        for (PIDGains gains : candidates) {
            assertEquals(0.001, gains.kF, 1e-12);
        }
    }

    @Test
    void feedforwardCharacterizerFitsSlopeThroughOrigin() {
        FeedforwardCharacterizer ff = new FeedforwardCharacterizer();

        // Simulate a perfectly linear motor: power = 0.0005 * velocity
        double trueKf = 0.0005;
        ff.addSample(0.5, 0.5 / trueKf);
        ff.addSample(0.75, 0.75 / trueKf);
        ff.addSample(1.0, 1.0 / trueKf);

        assertEquals(trueKf, ff.computeKf(), 1e-9);
    }
}
