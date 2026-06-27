package com.aaravdhawan25.pidautotuner;

import java.util.ArrayList;
import java.util.List;

/**
 * takes the Ku and Tu from the relay test and turns them into
 * actual pid gains using ziegler nichols rules
 *
 * basically there are like 100 years of control theory research that
 * figured out these formulas so we just use them lol
 *
 * reference: ziegler and nichols 1942 (yeah 1942, still works tho)
 * and astrom hagglund 1984 for the relay method part
 */
public class ZieglerNicholsCalculator {

    private ZieglerNicholsCalculator() {}

    /**
     * same as the other computeCandidates but defaults to including integral
     * (which you probably dont want for flywheels btw, use the other one
     * and pass false for includeIntegral)
     */
    public static List<PIDGains> computeCandidates(RelayTuningResult result, double kF) {
        return computeCandidates(result, kF, true);
    }

    /**
     * this is the main function, gives you 6 different gain options
     * from most conservative to most aggressive
     *
     * kF just gets passed through to every candidate since relay feedback
     * doesnt measure feedforward, you have to get that separately from
     * the feedforward sweep
     *
     * @param result          the Ku and Tu from RelayAutoTuner
     * @param kF              feedforward gain (0 if youre not using it)
     * @param includeIntegral false = all kI values are 0 (PD only rules).
     *                        for flywheels you almost always want false here
     *                        because kF handles steady state and integral
     *                        just causes windup which is really annoying
     */
    public static List<PIDGains> computeCandidates(RelayTuningResult result, double kF, boolean includeIntegral) {
        double Ku = result.Ku;
        double Tu = result.Tu;

        List<PIDGains> candidates = new ArrayList<>();

        if (!includeIntegral) {
            // PD only rules (no integral at all)
            // index 2 is "no overshoot" and index 4 is "classic ZN"
            // keeping them at the same indexes as the PID list below so
            // the code that grabs candidates.get(4) still works right

            // just proportional, no d term, usually has steady state error
            candidates.add(pid("P only", 0.5 * Ku, 0, 0, kF));

            // very gentle, barely does anything tbh
            candidates.add(pid("PD (very gentle)", 0.3 * Ku, 0, 0.3 * Ku * (Tu / 4.0), kF));

            // good for arms and lifts where you really dont want overshoot
            candidates.add(pid("PD (no overshoot)", 0.4 * Ku, 0, 0.4 * Ku * (Tu / 3.0), kF));

            // slightly more aggressive, still pretty safe
            candidates.add(pid("PD (some overshoot)", 0.6 * Ku, 0, 0.6 * Ku * (Tu / 4.0), kF));

            // this is the classic ziegler nichols PD formula, works great for flywheels
            candidates.add(pid("PD (classic ZN)", 0.8 * Ku, 0, 0.8 * Ku * (Tu / 8.0), kF));

            // go fast and break things basically lol, test carefully
            candidates.add(pid("PD (aggressive)", 1.2 * Ku, 0, 1.2 * Ku * (Tu / 8.0), kF));

            return candidates;
        }

        // PID rules (has integral term)
        // only use these if you set TUNE_INTEGRAL_TERM=true in TuningConfig

        // just P, boring but sometimes its all you need
        candidates.add(pid("P only", 0.5 * Ku, 0, 0, kF));

        // PI - adds integral to fix steady state error but no derivative
        {
            double Kp = 0.45 * Ku;
            double Ti = Tu / 1.2;
            candidates.add(pid("PI", Kp, Kp / Ti, 0, kF));
        }

        // smoothest full PID option, basically no overshoot at all
        {
            double Kp = 0.2 * Ku;
            double Ti = Tu / 2.0;
            double Td = Tu / 3.0;
            candidates.add(pid("PID (no overshoot)", Kp, Kp / Ti, Kp * Td, kF));
        }

        // a bit faster, small overshoot but settles quick
        {
            double Kp = 0.33 * Ku;
            double Ti = Tu / 2.0;
            double Td = Tu / 3.0;
            candidates.add(pid("PID (some overshoot)", Kp, Kp / Ti, Kp * Td, kF));
        }

        // the OG ziegler nichols formula, fast but kind of aggressive
        {
            double Kp = 0.6 * Ku;
            double Ti = Tu / 2.0;
            double Td = Tu / 8.0;
            candidates.add(pid("PID (classic ZN)", Kp, Kp / Ti, Kp * Td, kF));
        }

        // pessen integral rule, fastest response but most overshoot
        // definitely test this with the live test before using it in a match lol
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
