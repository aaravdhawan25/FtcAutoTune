package com.aaravdhawan25.pidautotuner.ftc;

import com.aaravdhawan25.pidautotuner.FeedforwardCharacterizer;
import com.aaravdhawan25.pidautotuner.PIDFController;
import com.aaravdhawan25.pidautotuner.PIDGains;
import com.aaravdhawan25.pidautotuner.RelayAutoTuner;
import com.aaravdhawan25.pidautotuner.RelayTuningResult;
import com.aaravdhawan25.pidautotuner.ZieglerNicholsCalculator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

/**
 * same as PIDMaster but for TWO motors at once
 * both motors get the exact same power output throughout the whole tune
 * so the gains work equally well for both
 *
 * designed for dual flywheel shooters and stuff like that where you have
 * two motors facing each other (which is why REVERSED_2 is usually true)
 *
 * if you only have one encoder plugged in (DUAL_ENCODERS=false), we just
 * read from motor 1's encoder and motor 2 gets the same power but we
 * ignore its encoder. this is totally fine and actually the common setup.
 *
 * important: we use Math.abs() on all velocity readings here
 * this is why this class worked fine even before we fixed PIDMaster lol
 */
public class DualMotorPIDMaster {

    // same phase enum as PIDMaster, just tracks where we are in the tuning
    private enum Phase {
        RELAY_TEST,
        FEEDFORWARD_SWEEP,
        RESULTS,
        TIMED_OUT,
        FAILED
    }

    private final DcMotorEx motor1;
    private final DcMotorEx motor2;

    private final double targetVelocity;
    private final double relayAmplitude;
    private final int cyclesToCollect;
    private final int cyclesToIgnore;
    private final double relayTestTimeoutS;
    private final double[] feedforwardTestPowers;
    private final double feedforwardSettleTimeS;
    private final boolean tuneIntegralTerm;
    private final boolean dualEncoders;

    private final RelayAutoTuner tuner;

    private Phase phase = Phase.RELAY_TEST;

    // feedforward sweep tracking
    private final FeedforwardCharacterizer ff = new FeedforwardCharacterizer();
    private int ffPowerIndex = 0;
    private double ffPhaseStartTime = -1;
    private double ffLastMeasurement = 0;

    // results
    private RelayTuningResult result;
    private List<PIDGains> candidates;
    private double kF = 0.0;

    // live test
    private PIDFController liveController;
    private boolean liveTestRunning = false;

    /**
     * sets everything up for the two motor tuning
     *
     * @param hardwareMap            from the opmode
     * @param motorName1             first motor hardware config name
     * @param reversed1              reverse motor 1 if needed
     * @param motorName2             second motor hardware config name
     * @param reversed2              usually true for opposing face flywheels
     * @param targetVelocity         target speed in ticks/sec for both motors
     * @param hysteresis             relay deadband in ticks/sec
     * @param relayAmplitude         bang-bang power magnitude (0-1)
     * @param cyclesToCollect        oscillation cycles to average
     * @param cyclesToIgnore         settling cycles to skip
     * @param relayTestTimeoutS      give up if it doesnt oscillate by this time
     * @param feedforwardTestPowers  powers to test for kF
     * @param feedforwardSettleTimeS wait this long at each power before measuring
     * @param tuneIntegralTerm       false = PD only (almost always what you want)
     * @param dualEncoders           true = both motors have encoders and we average them.
     *                               false = only motor 1 has encoder, motor 2 is just driven.
     *                               if you only have one encoder plugged in use false here
     */
    public DualMotorPIDMaster(HardwareMap hardwareMap,
                               String motorName1, boolean reversed1,
                               String motorName2, boolean reversed2,
                               double targetVelocity, double hysteresis,
                               double relayAmplitude, int cyclesToCollect, int cyclesToIgnore,
                               double relayTestTimeoutS,
                               double[] feedforwardTestPowers, double feedforwardSettleTimeS,
                               boolean tuneIntegralTerm, boolean dualEncoders) {

        this.targetVelocity = targetVelocity;
        this.relayAmplitude = relayAmplitude;
        this.cyclesToCollect = cyclesToCollect;
        this.cyclesToIgnore = cyclesToIgnore;
        this.relayTestTimeoutS = relayTestTimeoutS;
        this.feedforwardTestPowers = feedforwardTestPowers;
        this.feedforwardSettleTimeS = feedforwardSettleTimeS;
        this.tuneIntegralTerm = tuneIntegralTerm;
        this.dualEncoders = dualEncoders;

        motor1 = hardwareMap.get(DcMotorEx.class, motorName1);
        motor1.setDirection(reversed1
                ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor2 = hardwareMap.get(DcMotorEx.class, motorName2);
        motor2.setDirection(reversed2
                ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        tuner = new RelayAutoTuner(
                targetVelocity, relayAmplitude, hysteresis, cyclesToCollect, cyclesToIgnore);
    }

    /**
     * gets the velocity measurement
     * if both encoders: average of both motors (more accurate)
     * if single encoder: just motor 1 (totally fine for most setups)
     *
     * always uses Math.abs() so negative velocity from brief reversals
     * doesnt mess up the tuning
     */
    private double measure() {
        if (dualEncoders) {
            return (Math.abs(motor1.getVelocity()) + Math.abs(motor2.getVelocity())) / 2.0;
        } else {
            return Math.abs(motor1.getVelocity());
        }
    }

    /** sets both motors to the same power at the same time */
    private void setPower(double power) {
        motor1.setPower(power);
        motor2.setPower(power);
    }

    // -------------------------------------------------------------------------
    // state checks
    // -------------------------------------------------------------------------

    public boolean isTuningComplete() {
        return phase == Phase.RESULTS || phase == Phase.TIMED_OUT || phase == Phase.FAILED;
    }

    public boolean isTuningSuccessful() {
        return phase == Phase.RESULTS;
    }

    public boolean timedOut() {
        return phase == Phase.TIMED_OUT;
    }

    /**
     * call every loop while tuning is running
     * handles everything automatically
     *
     * @param now getRuntime() from your opmode
     */
    public void tuningStep(double now) {
        switch (phase) {
            case RELAY_TEST: {
                double measurement = measure();
                double output = tuner.update(measurement, now);
                setPower(output); // both motors get the same output

                if (tuner.isFinished()) {
                    setPower(0);
                    result = tuner.computeResult();
                    if (result == null) {
                        phase = Phase.FAILED;
                    } else if (feedforwardTestPowers != null && feedforwardTestPowers.length > 0) {
                        phase = Phase.FEEDFORWARD_SWEEP;
                        ffPowerIndex = 0;
                        ffPhaseStartTime = now;
                    } else {
                        finishTuning();
                    }
                } else if (now > relayTestTimeoutS) {
                    setPower(0);
                    phase = Phase.TIMED_OUT;
                }
                break;
            }

            case FEEDFORWARD_SWEEP: {
                double power = feedforwardTestPowers[ffPowerIndex];
                setPower(power); // again, same power to both
                ffLastMeasurement = measure();

                if (now - ffPhaseStartTime >= feedforwardSettleTimeS) {
                    ff.addSample(power, ffLastMeasurement);
                    ffPowerIndex++;
                    if (ffPowerIndex >= feedforwardTestPowers.length) {
                        setPower(0);
                        finishTuning();
                    } else {
                        ffPhaseStartTime = now;
                    }
                }
                break;
            }

            case RESULTS:
            case TIMED_OUT:
            case FAILED:
                break;
        }
    }

    private void finishTuning() {
        double rawKf = ff.computeKf();
        kF = Double.isNaN(rawKf) ? 0.0 : rawKf;
        candidates = ZieglerNicholsCalculator.computeCandidates(result, kF, tuneIntegralTerm);

        // index 4 = classic ZN, good default for flywheels
        PIDGains liveTestGains = candidates.get(4);
        liveController = PIDFController.fromGains(liveTestGains);
        liveController.setOutputBounds(-1.0, 1.0);

        phase = Phase.RESULTS;
    }

    // -------------------------------------------------------------------------
    // telemetry
    // -------------------------------------------------------------------------

    public List<String> getTelemetryLines() {
        List<String> lines = new ArrayList<>();
        switch (phase) {
            case RELAY_TEST:
                lines.add("=== Phase 1: Relay Test (Dual Motor) ===");
                lines.add(String.format("Encoder mode: %s", dualEncoders ? "DUAL (avg of both)" : "SINGLE (motor 1 only)"));
                lines.add(String.format("Measurement (ticks/s): %.1f", measure()));
                lines.add(String.format("Motor 1 velocity: %.1f", Math.abs(motor1.getVelocity())));
                if (dualEncoders) {
                    lines.add(String.format("Motor 2 velocity: %.1f", Math.abs(motor2.getVelocity())));
                } else {
                    lines.add("Motor 2 velocity: (no encoder plugged in)");
                }
                lines.add(String.format("Target (ticks/s): %.1f", targetVelocity));
                lines.add(String.format("Cycles collected: %d / %d",
                        tuner.cyclesCollected(), cyclesToCollect + cyclesToIgnore));
                break;
            case FEEDFORWARD_SWEEP:
                lines.add("=== Phase 2: Feedforward Sweep (Dual Motor) ===");
                lines.add(String.format("Encoder mode: %s", dualEncoders ? "DUAL" : "SINGLE (motor 1)"));
                lines.add(String.format("Testing power: %.2f", feedforwardTestPowers[ffPowerIndex]));
                lines.add(String.format("Measurement (ticks/s): %.1f", ffLastMeasurement));
                if (dualEncoders) {
                    lines.add(String.format("Motor 1: %.1f  Motor 2: %.1f",
                            Math.abs(motor1.getVelocity()), Math.abs(motor2.getVelocity())));
                } else {
                    lines.add(String.format("Motor 1: %.1f  Motor 2: (no encoder)",
                            Math.abs(motor1.getVelocity())));
                }
                break;
            case TIMED_OUT:
                lines.add("=== Tuning TIMED OUT ===");
                lines.add("Didn't oscillate enough.");
                lines.add("Try: increase RELAY_AMPLITUDE, lower target velocity,");
                lines.add("or check wiring on both motors.");
                break;
            case FAILED:
                lines.add("=== Tuning FAILED ===");
                lines.add("Not enough amplitude. Try increasing RELAY_AMPLITUDE.");
                break;
            case RESULTS:
                lines.addAll(getResultTelemetryLines());
                break;
        }
        return lines;
    }

    public List<String> getResultTelemetryLines() {
        List<String> lines = new ArrayList<>();
        if (result == null || candidates == null) {
            lines.add("(no result yet)");
            return lines;
        }

        lines.add("=== Relay Test Result (Dual Motor) ===");
        lines.add(String.format("Encoder mode: %s", dualEncoders ? "DUAL" : "SINGLE (motor 1 only)"));
        lines.add(String.format("Ku=%.6f  Tu=%.4fs", result.Ku, result.Tu));
        lines.add(String.format("kF=%.6f (samples: %d)", kF, ff.sampleCount()));
        lines.add("");
        lines.add("=== Candidate Gains (same for both motors) ===");
        for (PIDGains gains : candidates) {
            lines.add(gains.toString());
        }
        lines.add("");
        lines.add("These gains go on BOTH motors.");
        lines.add("Hold A to live-test classic ZN on both motors.");
        return lines;
    }

    // -------------------------------------------------------------------------
    // live test
    // -------------------------------------------------------------------------

    /**
     * runs the live test with classic ZN gains
     * both motors run at the same time with the same output
     *
     * @param now getRuntime()
     * @return telemetry lines
     */
    public List<String> liveTestStep(double now) {
        List<String> lines = new ArrayList<>();
        if (liveController == null) {
            lines.add("(can't live test - tuning not done yet)");
            return lines;
        }

        if (!liveTestRunning) {
            liveController.reset();
            liveTestRunning = true;
        }

        double measurement = measure();
        double output = liveController.calculate(targetVelocity, measurement, now);
        setPower(output); // same power to both motors

        lines.add("=== LIVE TEST ACTIVE (Dual Motor) ===");
        lines.add(String.format("Measurement: %.1f  Target: %.1f  Error: %.1f",
                measurement, targetVelocity, targetVelocity - measurement));
        if (dualEncoders) {
            lines.add(String.format("Motor 1: %.1f  Motor 2: %.1f",
                    Math.abs(motor1.getVelocity()), Math.abs(motor2.getVelocity())));
        } else {
            lines.add(String.format("Motor 1: %.1f  Motor 2: (no encoder)",
                    Math.abs(motor1.getVelocity())));
        }
        lines.add(String.format("Output power: %.3f", output));
        return lines;
    }

    public void stopLiveTest() {
        if (liveTestRunning) {
            setPower(0);
            liveTestRunning = false;
        }
    }

    public void stop() {
        setPower(0);
    }
}
