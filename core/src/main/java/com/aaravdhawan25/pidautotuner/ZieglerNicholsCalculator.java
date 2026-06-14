package com.aaravdhawan25.pidautotuner;

import java.util.ArrayList;
import java.util.List;

/**
 * Converts an ultimate gain {@code Ku} and ultimate period {@code Tu}
 * (as produced by {@link RelayAutoTuner}) into one or more candidate sets of
 * PID(F) gains, using the classic Ziegler-Nichols closed-loop rules and a
 * couple of common variants.
 *
 * <p>All rules below follow the standard form:
 * <pre>
 *     Kp = a * Ku
 *     Ki = Kp / Ti
 *     Kd = Kp * Td
 * </pre>
 * where {@code Ti} and {@code Td} are given as fractions of {@code Tu}.
 *
 * <p>Reference: Ziegler, J.G. &amp; Nichols, N.B. (1942), "Optimum Settings for
 * Automatic Controllers"; Astrom &amp; Hagglund (1984) for the relay method
 * that produces Ku/Tu in the first place.
 */
public class ZieglerNicholsCalculator {

    private ZieglerNicholsCalculator() {}

    /**
     * Produces a set of candidate PID gains. {@code kF} is passed through
     * unchanged on every candidate -- it is determined separately (see
     * {@link FeedforwardCharacterizer}) since relay feedback does not measure
     * feedforward terms.
     *
     * @param result the output of a completed {@link RelayAutoTuner} test
     * @param kF     feedforward gain to apply to every candidate (0 if not used)
     * @return a list of labeled candidate gain sets, roughly ordered from
     *         most conservative to most aggressive
     */
    public static List<PIDGains> computeCandidates(RelayTuningResult result, double kF) {
        return computeCandidates(result, kF, true);
    }

    /**
     * Same as {@link #computeCandidates(RelayTuningResult, double)}, but lets
     * you disable the integral term entirely. When {@code includeIntegral} is
     * {@code false}, every candidate has {@code kI = 0} and the candidates
     * are computed using the Ziegler-Nichols <b>PD</b> rule family instead of
     * the PID family -- this is a common choice for velocity/flywheel loops
     * where a feedforward term ({@code kF}) already handles steady-state
     * error, and an integral term would mainly add windup risk.
     *
     * @param result          the output of a completed {@link RelayAutoTuner} test
     * @param kF              feedforward gain to apply to every candidate (0 if not used)
     * @param includeIntegral if false, all candidates use kI = 0 (PD-only rules)
     * @return a list of labeled candidate gain sets, roughly ordered from
     *         most conservative to most aggressive
     */
    public static List<PIDGains> computeCandidates(RelayTuningResult result, double kF, boolean includeIntegral) {
        double Ku = result.Ku;
        double Tu = result.Tu;

        List<PIDGains> candidates = new ArrayList<>();

        if (!includeIntegral) {
            // Ziegler-Nichols PD-family rules (kI = 0 throughout). Ordered so
            // index 2 is "no overshoot" and index 4 is "classic ZN", matching
            // the PID-mode list below -- callers that do candidates.get(2) or
            // candidates.get(4) get the conceptually equivalent entry either way.

            // P only
            candidates.add(pid("P only", 0.5 * Ku, 0, 0, kF));

            // PD (very gentle)
            candidates.add(pid("PD (very gentle)", 0.3 * Ku, 0, 0.3 * Ku * (Tu / 4.0), kF));

            // PD (no overshoot) -- gentlest full PD rule, good for arms/lifts
            candidates.add(pid("PD (no overshoot)", 0.4 * Ku, 0, 0.4 * Ku * (Tu / 3.0), kF));

            // PD (some overshoot)
            candidates.add(pid("PD (some overshoot)", 0.6 * Ku, 0, 0.6 * Ku * (Tu / 4.0), kF));

            // PD (classic ZN) -- standard ZN PD rule: Kp = 0.8*Ku, Td = Tu/8
            candidates.add(pid("PD (classic ZN)", 0.8 * Ku, 0, 0.8 * Ku * (Tu / 8.0), kF));

            // PD (aggressive)
            candidates.add(pid("PD (aggressive)", 1.2 * Ku, 0, 1.2 * Ku * (Tu / 8.0), kF));

            return candidates;
        }

        // P only
        candidates.add(pid("P only", 0.5 * Ku, 0, 0, kF));

        // PI
        {
            double Kp = 0.45 * Ku;
            double Ti = Tu / 1.2;
            candidates.add(pid("PI", Kp, Kp / Ti, 0, kF));
        }

        // No-overshoot PID (gentlest full PID rule -- good starting point for FTC)
        {
            double Kp = 0.2 * Ku;
            double Ti = Tu / 2.0;
            double Td = Tu / 3.0;
            candidates.add(pid("PID (no overshoot)", Kp, Kp / Ti, Kp * Td, kF));
        }

        // "Some overshoot" PID
        {
            double Kp = 0.33 * Ku;
            double Ti = Tu / 2.0;
            double Td = Tu / 3.0;
            candidates.add(pid("PID (some overshoot)", Kp, Kp / Ti, Kp * Td, kF));
        }

        // Classic ZN PID
        {
            double Kp = 0.6 * Ku;
            double Ti = Tu / 2.0;
            double Td = Tu / 8.0;
            candidates.add(pid("PID (classic ZN)", Kp, Kp / Ti, Kp * Td, kF));
        }

        // Pessen Integral Rule (most aggressive -- fastest response, more overshoot)
        {
            double Kp = 0.7 * Ku;
            double Ti = 0.4 * Tu;
            double Td = 0.15 * Tu;
            candidates.add(pid("PID (Pessen, aggressive)", Kp, Kp / Ti, Kp * Td, kF));
        }

        return candidates;
    }

    private static PIDGains pid(String label, double kP, double kI, double kD, double kF) {
        return new PIDGains(label, kP, kI, kD, kF);
    }
}
