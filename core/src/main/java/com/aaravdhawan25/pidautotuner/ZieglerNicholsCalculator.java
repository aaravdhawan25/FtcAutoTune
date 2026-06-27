package com.aaravdhawan25.pidautotuner;

import java.util.ArrayList;
import java.util.List;

/**
 * Converts the ultimate gain {@code Ku} and ultimate period {@code Tu} produced
 * by {@link RelayAutoTuner} into candidate PID(F) gain sets using the
 * Ziegler–Nichols closed-loop tuning rules and common variants thereof.
 *
 * <p>All rules follow the standard parameterisation:
 * <pre>
 *   Kp = α · Ku
 *   Ki = Kp / Ti     where Ti is a fraction of Tu
 *   Kd = Kp · Td     where Td is a fraction of Tu
 * </pre>
 *
 * <p>References:
 * <ul>
 *   <li>Ziegler, J.G. and Nichols, N.B. (1942). "Optimum Settings for Automatic
 *       Controllers." <i>Transactions of the ASME</i>, 64, 759–768.</li>
 *   <li>Åström, K.J. and Hägglund, T. (1984). "Automatic tuning of simple
 *       regulators with specifications on phase and amplitude margins."
 *       <i>Automatica</i>, 20(5), 645–651.</li>
 * </ul>
 */
public class ZieglerNicholsCalculator {

    private ZieglerNicholsCalculator() {}

    /**
     * Computes candidate gain sets using the default behaviour of
     * {@link #computeCandidates(RelayTuningResult, double, boolean)},
     * with integral terms included.
     *
     * @param result the output of a completed {@link RelayAutoTuner} test
     * @param kF     feedforward gain applied to every candidate (0 if unused)
     * @return a list of six labeled gain sets ordered from conservative to aggressive
     */
    public static List<PIDGains> computeCandidates(RelayTuningResult result, double kF) {
        return computeCandidates(result, kF, true);
    }

    /**
     * Computes six candidate PID(F) gain sets from the relay-test result.
     *
     * <p>When {@code includeIntegral} is {@code false}, the PD-only
     * Ziegler–Nichols rule family is used and every candidate has
     * {@code kI = 0}. This is the recommended setting for velocity and
     * flywheel loops, where the feedforward term {@code kF} already
     * eliminates steady-state error and an integral term primarily
     * introduces windup risk.
     *
     * <p>The candidate list is ordered so that index 2 is the "no overshoot"
     * variant and index 4 is the "classic ZN" variant in both the PD-only
     * and full-PID families, allowing callers to reference a consistent
     * index regardless of the {@code includeIntegral} setting.
     *
     * @param result          the output of a completed {@link RelayAutoTuner} test
     * @param kF              feedforward gain applied to every candidate (0 if unused)
     * @param includeIntegral {@code false} to use PD-only rules with {@code kI = 0};
     *                        {@code true} to use the full PID rule family
     * @return a list of six labeled gain sets ordered from conservative to aggressive
     */
    public static List<PIDGains> computeCandidates(RelayTuningResult result, double kF, boolean includeIntegral) {
        double Ku = result.Ku;
        double Tu = result.Tu;

        List<PIDGains> candidates = new ArrayList<>();

        if (!includeIntegral) {
            // PD-only rule family (kI = 0 throughout).
            // Index alignment: 0 = P only, 1 = very gentle, 2 = no overshoot,
            //                  3 = some overshoot, 4 = classic ZN, 5 = aggressive.
            candidates.add(pid("P only",              0.5  * Ku, 0, 0,                        kF));
            candidates.add(pid("PD (very gentle)",    0.3  * Ku, 0, 0.3  * Ku * (Tu / 4.0),  kF));
            candidates.add(pid("PD (no overshoot)",   0.4  * Ku, 0, 0.4  * Ku * (Tu / 3.0),  kF));
            candidates.add(pid("PD (some overshoot)", 0.6  * Ku, 0, 0.6  * Ku * (Tu / 4.0),  kF));
            candidates.add(pid("PD (classic ZN)",     0.8  * Ku, 0, 0.8  * Ku * (Tu / 8.0),  kF));
            candidates.add(pid("PD (aggressive)",     1.2  * Ku, 0, 1.2  * Ku * (Tu / 8.0),  kF));
            return candidates;
        }

        // Full PID rule family.
        // Index alignment mirrors the PD family: 0 = P only, 2 = no overshoot,
        //                                        4 = classic ZN, 5 = Pessen.
        candidates.add(pid("P only", 0.5 * Ku, 0, 0, kF));

        {
            double Kp = 0.45 * Ku;
            double Ti = Tu / 1.2;
            candidates.add(pid("PI", Kp, Kp / Ti, 0, kF));
        }
        {
            double Kp = 0.2 * Ku;
            double Ti = Tu / 2.0;
            double Td = Tu / 3.0;
            candidates.add(pid("PID (no overshoot)", Kp, Kp / Ti, Kp * Td, kF));
        }
        {
            double Kp = 0.33 * Ku;
            double Ti = Tu / 2.0;
            double Td = Tu / 3.0;
            candidates.add(pid("PID (some overshoot)", Kp, Kp / Ti, Kp * Td, kF));
        }
        {
            double Kp = 0.6 * Ku;
            double Ti = Tu / 2.0;
            double Td = Tu / 8.0;
            candidates.add(pid("PID (classic ZN)", Kp, Kp / Ti, Kp * Td, kF));
        }
        {
            // Pessen Integral Rule — highest bandwidth, most overshoot.
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
