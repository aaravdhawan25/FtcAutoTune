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
        double Ku = result.Ku;
        double Tu = result.Tu;

        List<PIDGains> candidates = new ArrayList<>();

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
