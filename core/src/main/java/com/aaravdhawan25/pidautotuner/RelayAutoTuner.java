package com.aaravdhawan25.pidautotuner;

import java.util.ArrayList;
import java.util.List;

/**
 * ok so this is the main tuning engine basically
 * it does the relay (bang-bang) thing where it slams the motor full power
 * one way until the speed/position goes past the target, then slams it
 * the other way. this makes it oscillate and from that we can figure out
 * the pid gains. its actually really clever ngl
 *
 * based on the astrom hagglund method from 1984 which sounds old but
 * it still works great for ftc motors so whatever
 *
 * this class has literally no idea what a motor is, it just takes numbers
 * in and spits numbers out. you have to do the actual motor stuff yourself:
 * 1. read your encoder
 * 2. call update() with that number and the current time
 * 3. set your motor power to whatever update() returns
 * 4. keep doing that until isFinished() is true
 * 5. call computeResult() to get Ku and Tu
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
     * sets up the tuner, pretty self explanatory
     *
     * @param setpoint        the speed or position you want to oscillate around
     * @param relayAmplitude  how hard to slam the motor (0 to 1). like 0.5 is
     *                        usually fine but go lower for arms so they dont
     *                        destroy themselves
     * @param hysteresis      a tiny deadband so random noise doesnt make it
     *                        switch back and forth super fast. like 10 ticks
     *                        for position or 30 ticks/sec for velocity
     * @param cyclesToCollect how many full oscillations to actually use for
     *                        the math. more = more accurate but takes longer.
     *                        6 is good
     * @param cyclesToIgnore  skip the first few cycles because the system is
     *                        still warming up and the data is kinda garbage.
     *                        2 is usually enough
     */
    public RelayAutoTuner(double setpoint, double relayAmplitude, double hysteresis,
                           int cyclesToCollect, int cyclesToIgnore) {
        if (relayAmplitude <= 0) {
            throw new IllegalArgumentException("relayAmplitude has to be positive bro");
        }
        if (hysteresis < 0) {
            throw new IllegalArgumentException("hysteresis cant be negative that makes no sense");
        }
        this.setpoint = setpoint;
        this.relayAmplitude = relayAmplitude;
        this.hysteresis = hysteresis;
        this.cyclesToCollect = cyclesToCollect;
        this.cyclesToIgnore = cyclesToIgnore;
    }

    /**
     * call this every loop. give it your current measurement and the time,
     * it tells you what power to set the motor to.
     *
     * @param measurement      whatever youre measuring (ticks or ticks/sec)
     * @param timestampSeconds current time in seconds, just use getRuntime()
     * @return either +relayAmplitude or -relayAmplitude, set your motor to this
     */
    public double update(double measurement, double timestampSeconds) {
        // track the highest/lowest point since the last switch
        // so we can figure out how big the oscillation is
        if (!hasExtremum) {
            currentExtremum = measurement;
            hasExtremum = true;
        } else if (outputHigh) {
            currentExtremum = Math.max(currentExtremum, measurement);
        } else {
            currentExtremum = Math.min(currentExtremum, measurement);
        }

        // this is where we actually decide to flip the relay
        // going high -> switch to low when we pass the top threshold
        // going low -> switch to high when we pass the bottom threshold
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
     * returns true when we have enough data to compute the gains
     * one "cycle" is a full oscillation (up half + down half)
     */
    public boolean isFinished() {
        int usableHalfPeriods = (cyclesToCollect + cyclesToIgnore) * 2;
        return halfPeriods.size() >= usableHalfPeriods;
    }

    /** how many full oscillation cycles weve recorded so far */
    public int cyclesCollected() {
        return halfPeriods.size() / 2;
    }

    /**
     * call this after isFinished() is true to get Ku and Tu
     * which then get fed into ZieglerNicholsCalculator to get actual pid gains
     *
     * returns null if we somehow dont have enough data (shouldnt happen if
     * you waited for isFinished() but just in case)
     */
    public RelayTuningResult computeResult() {
        int ignoreHalfPeriods = cyclesToIgnore * 2;
        int usableHalfPeriods = cyclesToCollect * 2;

        if (halfPeriods.size() < ignoreHalfPeriods + usableHalfPeriods) {
            return null;
        }

        // average the period across all the good cycles
        // one full cycle = two half periods (high + low added together)
        double periodSum = 0;
        int periodCount = 0;
        for (int i = ignoreHalfPeriods; i < ignoreHalfPeriods + usableHalfPeriods; i += 2) {
            periodSum += halfPeriods.get(i) + halfPeriods.get(i + 1);
            periodCount++;
        }
        double Tu = periodSum / periodCount;

        // figure out the average amplitude
        // the extrema list alternates between peaks and troughs
        // so peak-to-trough / 2 gives us the amplitude 'a'
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

        // this is the astrom-hagglund formula for ultimate gain
        // Ku = 4d / (pi * a) where d is relay amplitude and a is oscillation amplitude
        double Ku = (4.0 * relayAmplitude) / (Math.PI * a);

        return new RelayTuningResult(Ku, Tu, a, relayAmplitude);
    }
}
