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
 * Drives the full relay-feedback velocity auto-tuning workflow for a
 * <b>dual-motor</b> mechanism (e.g. a two-flywheel shooter, dual intake
 * rollers, or a differential-drive velocity loop).
 *
 * <p>Both motors are driven with the same output throughout the relay test
 * and feedforward sweep. The relay test uses the <b>average absolute
 * velocity</b> of both motors as the measurement, so both wheels must be
 * spinning to get a valid tune. The resulting kP/kI/kD/kF are applied
 * identically to both motors in the live test.
 *
 * <p>Each motor has its own direction setting, since on flywheel/shooter
 * setups the two motors typically face each other and one must be reversed.
 *
 * <h2>Usage</h2>
 * <pre>{@code
 * DualMotorPIDMaster pid = new DualMotorPIDMaster(
 *         hardwareMap,
 *         TuningConfig.MOTOR_NAME,   TuningConfig.REVERSED,
 *         TuningConfig.MOTOR_NAME_2, TuningConfig.REVERSED_2,
 *         TuningConfig.VELOCITY_TARGET_TICKS_PER_SEC,
 *         TuningConfig.VELOCITY_HYSTERESIS_TICKS_PER_SEC,
 *         TuningConfig.RELAY_AMPLITUDE,
 *         TuningConfig.CYCLES_TO_COLLECT, TuningConfig.CYCLES_TO_IGNORE,
 *         TuningConfig.RELAY_TEST_TIMEOUT_S,
 *         TuningConfig.FEEDFORWARD_TEST_POWERS,
 *         TuningConfig.FEEDFORWARD_SETTLE_TIME_S,
 *         TuningConfig.TUNE_INTEGRAL_TERM);
 *
 * while (opModeIsActive() && !pid.isTuningComplete()) {
 *     pid.tuningStep(getRuntime());
 *     for (String line : pid.getTelemetryLines()) telemetry.addLine(line);
 *     telemetry.update();
 * }
 *
 * while (opModeIsActive()) {
 *     telemetry.clearAll();
 *     for (String line : pid.getResultTelemetryLines()) telemetry.addLine(line);
 *     if (gamepad1.a) pid.liveTestStep(getRuntime()); else pid.stopLiveTest();
 *     telemetry.update();
 * }
 * pid.stop();
 * }</pre>
 */
public class DualMotorPIDMaster {

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

    // Feedforward sweep state
    private final FeedforwardCharacterizer ff = new FeedforwardCharacterizer();
    private int ffPowerIndex = 0;
    private double ffPhaseStartTime = -1;
    private double ffLastMeasurement = 0;

    // Results
    private RelayTuningResult result;
    private List<PIDGains> candidates;
    private double kF = 0.0;

    // Live test
    private PIDFController liveController;
    private boolean liveTestRunning = false;

    /**
     * @param hardwareMap            from the OpMode
     * @param motorName1             hardware config name of the first motor
     * @param reversed1              true to reverse the first motor's direction
     * @param motorName2             hardware config name of the second motor
     * @param reversed2              true to reverse the second motor's direction
     * @param targetVelocity         target velocity in ticks/sec for both motors
     * @param hysteresis             deadband for the relay test in ticks/sec
     * @param relayAmplitude         relay output magnitude (0-1)
     * @param cyclesToCollect        oscillation cycles to average
     * @param cyclesToIgnore         initial cycles to discard
     * @param relayTestTimeoutS      safety timeout for the relay test
     * @param feedforwardTestPowers  open-loop powers for kF characterization
     * @param feedforwardSettleTimeS settle time per power level
     * @param tuneIntegralTerm       false = use PD-only rules (kI=0)
     * @param dualEncoders           true if both motors have encoders plugged in;
     *                               false if only motor 1 has an encoder (motor 2
     *                               is still driven but its velocity is not measured)
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
     * Returns the velocity measurement used by the relay tuner.
     * <ul>
     *     <li>If {@code dualEncoders = true}: average absolute velocity of both motors</li>
     *     <li>If {@code dualEncoders = false}: absolute velocity of motor 1 only
     *         (motor 2 still receives the same drive power, its encoder just isn't read)</li>
     * </ul>
     */
    private double measure() {
        if (dualEncoders) {
            return (Math.abs(motor1.getVelocity()) + Math.abs(motor2.getVelocity())) / 2.0;
        } else {
            return Math.abs(motor1.getVelocity());
        }
    }

    /** Applies the same power to both motors. */
    private void setPower(double power) {
        motor1.setPower(power);
        motor2.setPower(power);
    }

    // -------------------------------------------------------------------------
    // State / tuning
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
     * Call once per loop iteration while {@code !isTuningComplete()}.
     *
     * @param now monotonically increasing time in seconds (e.g. {@code getRuntime()})
     */
    public void tuningStep(double now) {
        switch (phase) {
            case RELAY_TEST: {
                double measurement = measure();
                double output = tuner.update(measurement, now);
                setPower(output);

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
                setPower(power);
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

        // index 4 = "classic ZN" -- good default for flywheel/velocity loops
        PIDGains liveTestGains = candidates.get(4);
        liveController = PIDFController.fromGains(liveTestGains);
        liveController.setOutputBounds(-1.0, 1.0);

        phase = Phase.RESULTS;
    }

    // -------------------------------------------------------------------------
    // Telemetry
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
                    lines.add("Motor 2 velocity: (no encoder)");
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
                lines.add("Didn't complete enough oscillation cycles.");
                lines.add("Try increasing RELAY_AMPLITUDE, adjusting the");
                lines.add("target velocity, or check wiring on both motors.");
                break;
            case FAILED:
                lines.add("=== Tuning FAILED ===");
                lines.add("Not enough oscillation amplitude was measured.");
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
        lines.add("=== Candidate Gains (apply to BOTH motors) ===");
        for (PIDGains gains : candidates) {
            lines.add(gains.toString());
        }
        lines.add("");
        lines.add("These gains run identically on both motors.");
        lines.add("Hold A to live-test 'classic ZN' on both motors.");
        return lines;
    }

    // -------------------------------------------------------------------------
    // Live test
    // -------------------------------------------------------------------------

    /**
     * Runs one iteration of the live test with the "classic ZN" candidate,
     * applying the same output to both motors simultaneously.
     *
     * @param now monotonically increasing time in seconds
     * @return telemetry lines for the live-test state
     */
    public List<String> liveTestStep(double now) {
        List<String> lines = new ArrayList<>();
        if (liveController == null) {
            lines.add("(live test unavailable -- tuning not complete)");
            return lines;
        }

        if (!liveTestRunning) {
            liveController.reset();
            liveTestRunning = true;
        }

        double measurement = measure();
        double output = liveController.calculate(targetVelocity, measurement, now);
        setPower(output);

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
