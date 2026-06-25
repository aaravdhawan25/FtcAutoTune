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
 * Drives the full relay-feedback auto-tuning workflow (relay test, optional
 * feedforward sweep, candidate computation, and a live-test loop) for either
 * a run-to-position mechanism or a velocity-controlled mechanism.
 *
 * <p>This class lives in the {@code pidautotuner-ftc} library and is used by
 * a thin TeleOp that you copy into your own TeamCode (along with
 * {@code TuningConfig}). All configuration is passed in via the constructor
 * -- {@code PIDMaster} has no dependency on any class in your TeamCode.
 *
 * <h2>Usage (position mode)</h2>
 * <pre>{@code
 * PIDMaster pid = new PIDMaster(
 *         hardwareMap, TuningConfig.MOTOR_NAME, TuningConfig.REVERSED, true,
 *         TuningConfig.POSITION_TARGET_TICKS, TuningConfig.POSITION_HYSTERESIS_TICKS,
 *         TuningConfig.RELAY_AMPLITUDE, TuningConfig.CYCLES_TO_COLLECT, TuningConfig.CYCLES_TO_IGNORE,
 *         TuningConfig.RELAY_TEST_TIMEOUT_S, null, 0, TuningConfig.TUNE_INTEGRAL_TERM);
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
 *
 * <h2>Usage (velocity mode)</h2>
 * <p>Same as above, but pass {@code positionMode = false}, give the target
 * velocity (ticks/sec) as {@code targetValue}, and provide
 * {@code feedforwardTestPowers} / {@code feedforwardSettleTimeS} so the
 * feedforward sweep runs automatically after the relay test.
 */
public class PIDMaster {

    private enum Phase {
        RELAY_TEST,
        FEEDFORWARD_SWEEP,
        RESULTS,
        TIMED_OUT,
        FAILED
    }

    private final DcMotorEx motor;
    private final boolean positionMode;

    private final double relayAmplitude;
    private final int cyclesToCollect;
    private final int cyclesToIgnore;
    private final double relayTestTimeoutS;
    private final double[] feedforwardTestPowers;
    private final double feedforwardSettleTimeS;
    private final boolean tuneIntegralTerm;

    private final RelayAutoTuner tuner;
    private final double setpoint; // absolute setpoint for position mode; == targetValue for velocity mode

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

    // RPM display
    private final double ticksPerRev;

    /**
     * @param hardwareMap            from the OpMode
     * @param motorName              hardware config name of the motor, e.g. {@code TuningConfig.MOTOR_NAME}
     * @param reversed               {@code TuningConfig.REVERSED}
     * @param positionMode           true for run-to-position tuning, false for velocity/PIDF tuning
     * @param targetValue            for position mode: target offset in ticks from the start position
     *                               ({@code TuningConfig.POSITION_TARGET_TICKS}); for velocity mode:
     *                               target velocity in ticks/sec ({@code TuningConfig.VELOCITY_TARGET_TICKS_PER_SEC})
     * @param hysteresis             {@code TuningConfig.POSITION_HYSTERESIS_TICKS} or
     *                               {@code TuningConfig.VELOCITY_HYSTERESIS_TICKS_PER_SEC}
     * @param relayAmplitude         {@code TuningConfig.RELAY_AMPLITUDE}
     * @param cyclesToCollect        {@code TuningConfig.CYCLES_TO_COLLECT}
     * @param cyclesToIgnore         {@code TuningConfig.CYCLES_TO_IGNORE}
     * @param relayTestTimeoutS      {@code TuningConfig.RELAY_TEST_TIMEOUT_S}
     * @param feedforwardTestPowers  velocity mode only: {@code TuningConfig.FEEDFORWARD_TEST_POWERS}.
     *                               Pass {@code null} for position mode.
     * @param feedforwardSettleTimeS velocity mode only: {@code TuningConfig.FEEDFORWARD_SETTLE_TIME_S}.
     *                               Ignored for position mode.
     * @param tuneIntegralTerm       {@code TuningConfig.TUNE_INTEGRAL_TERM}
     * @param ticksPerRev            {@code TuningConfig.TICKS_PER_REV} -- used to display RPM
     *                               alongside ticks/sec in telemetry. Pass 0 to disable RPM display.
     */
    public PIDMaster(HardwareMap hardwareMap, String motorName, boolean reversed, boolean positionMode,
                      double targetValue, double hysteresis,
                      double relayAmplitude, int cyclesToCollect, int cyclesToIgnore, double relayTestTimeoutS,
                      double[] feedforwardTestPowers, double feedforwardSettleTimeS, boolean tuneIntegralTerm,
                      double ticksPerRev) {

        this.positionMode = positionMode;
        this.relayAmplitude = relayAmplitude;
        this.cyclesToCollect = cyclesToCollect;
        this.cyclesToIgnore = cyclesToIgnore;
        this.relayTestTimeoutS = relayTestTimeoutS;
        this.feedforwardTestPowers = feedforwardTestPowers;
        this.feedforwardSettleTimeS = feedforwardSettleTimeS;
        this.tuneIntegralTerm = tuneIntegralTerm;
        this.ticksPerRev = ticksPerRev;

        motor = hardwareMap.get(DcMotorEx.class, motorName);
        motor.setDirection(reversed
                ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(positionMode
                ? DcMotor.ZeroPowerBehavior.BRAKE
                : DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (positionMode) {
            int startPosition = motor.getCurrentPosition();
            setpoint = startPosition + targetValue;
        } else {
            setpoint = targetValue;
        }

        tuner = new RelayAutoTuner(setpoint, relayAmplitude, hysteresis, cyclesToCollect, cyclesToIgnore);
    }

    /** @return the current measurement: encoder position (position mode) or velocity in ticks/sec (velocity mode). */
    private double measure() {
        return positionMode ? motor.getCurrentPosition() : motor.getVelocity();
    }

    // ---------------------------------------------------------------------
    // Tuning (relay test + feedforward sweep)
    // ---------------------------------------------------------------------

    /** @return true once tuning has finished (successfully, timed out, or failed). */
    public boolean isTuningComplete() {
        return phase == Phase.RESULTS || phase == Phase.TIMED_OUT || phase == Phase.FAILED;
    }

    /** @return true if tuning finished successfully and {@link #getResultTelemetryLines()} has results. */
    public boolean isTuningSuccessful() {
        return phase == Phase.RESULTS;
    }

    /** @return true if the relay test exceeded {@code relayTestTimeoutS} without enough oscillation. */
    public boolean timedOut() {
        return phase == Phase.TIMED_OUT;
    }

    /**
     * Call once per loop iteration while {@code !isTuningComplete()}. Advances
     * the relay test and (for velocity mode) the feedforward sweep, applying
     * motor power as needed.
     *
     * @param now monotonically increasing time in seconds (e.g. {@code getRuntime()})
     */
    public void tuningStep(double now) {
        switch (phase) {
            case RELAY_TEST: {
                double measurement = measure();
                double output = tuner.update(measurement, now);
                motor.setPower(output);

                if (tuner.isFinished()) {
                    motor.setPower(0);
                    result = tuner.computeResult();
                    if (result == null) {
                        phase = Phase.FAILED;
                    } else if (!positionMode && feedforwardTestPowers != null && feedforwardTestPowers.length > 0) {
                        phase = Phase.FEEDFORWARD_SWEEP;
                        ffPowerIndex = 0;
                        ffPhaseStartTime = now;
                    } else {
                        finishTuning();
                    }
                } else if (now > relayTestTimeoutS) {
                    motor.setPower(0);
                    phase = Phase.TIMED_OUT;
                }
                break;
            }

            case FEEDFORWARD_SWEEP: {
                double power = feedforwardTestPowers[ffPowerIndex];
                motor.setPower(power);
                ffLastMeasurement = motor.getVelocity();

                if (now - ffPhaseStartTime >= feedforwardSettleTimeS) {
                    ff.addSample(power, ffLastMeasurement);
                    ffPowerIndex++;
                    if (ffPowerIndex >= feedforwardTestPowers.length) {
                        motor.setPower(0);
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
                // nothing to do
                break;
        }
    }

    private void finishTuning() {
        if (!positionMode) {
            double rawKf = ff.computeKf();
            kF = Double.isNaN(rawKf) ? 0.0 : rawKf;
        } else {
            kF = 0.0;
        }
        candidates = ZieglerNicholsCalculator.computeCandidates(result, kF, tuneIntegralTerm);

        // index 2 = "no overshoot" (position default), index 4 = "classic ZN" (velocity default)
        PIDGains liveTestGains = candidates.get(positionMode ? 2 : 4);
        liveController = PIDFController.fromGains(liveTestGains);
        liveController.setOutputBounds(-1.0, 1.0);

        phase = Phase.RESULTS;
    }

    // ---------------------------------------------------------------------
    // Telemetry
    // ---------------------------------------------------------------------

    /** Converts ticks/sec to RPM if ticksPerRev is set, otherwise returns null. */
    private String rpmString(double ticksPerSec) {
        if (ticksPerRev > 0) {
            return String.format("  (%.1f RPM)", (ticksPerSec / ticksPerRev) * 60.0);
        }
        return "";
    }

    /** @return telemetry lines describing tuning progress (relay test / feedforward sweep). Call while {@code !isTuningComplete()}. */
    public List<String> getTelemetryLines() {
        List<String> lines = new ArrayList<>();
        switch (phase) {
            case RELAY_TEST:
                lines.add("=== Phase 1: Relay Test ===");
                if (positionMode) {
                    lines.add(String.format("Position: %.0f", measure()));
                    lines.add(String.format("Setpoint: %.1f", setpoint));
                } else {
                    lines.add(String.format("Velocity: %.1f ticks/s%s", measure(), rpmString(measure())));
                    lines.add(String.format("Target:   %.1f ticks/s%s", setpoint, rpmString(setpoint)));
                }
                lines.add(String.format("Cycles collected: %d / %d", tuner.cyclesCollected(), cyclesToCollect + cyclesToIgnore));
                break;
            case FEEDFORWARD_SWEEP:
                lines.add("=== Phase 2: Feedforward Sweep ===");
                lines.add(String.format("Testing power: %.2f", feedforwardTestPowers[ffPowerIndex]));
                lines.add(String.format("Velocity: %.1f ticks/s%s", ffLastMeasurement, rpmString(ffLastMeasurement)));
                break;
            case TIMED_OUT:
                lines.add("=== Tuning TIMED OUT ===");
                lines.add("Didn't complete enough oscillation cycles.");
                lines.add("Try increasing RELAY_AMPLITUDE, adjusting the");
                lines.add("target value, or check wiring.");
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

    /** @return telemetry lines with the relay-test result and candidate gain sets. Call once {@code isTuningSuccessful()}. */
    public List<String> getResultTelemetryLines() {
        List<String> lines = new ArrayList<>();
        if (result == null || candidates == null) {
            lines.add("(no result yet)");
            return lines;
        }

        lines.add("=== Relay Test Result ===");
        lines.add(String.format("Ku=%.6f  Tu=%.4fs", result.Ku, result.Tu));
        if (!positionMode) {
            lines.add(String.format("kF=%.6f (samples: %d)", kF, ff.sampleCount()));
        }
        lines.add("");
        lines.add("=== Candidate Gains ===");
        for (PIDGains gains : candidates) {
            lines.add(gains.toString());
        }
        lines.add("");
        lines.add("Copy a candidate's kP/kI/kD/kF into your code.");
        lines.add(positionMode
                ? "Hold A to live-test 'no overshoot'."
                : "Hold A to live-test 'classic ZN'.");
        return lines;
    }

    // ---------------------------------------------------------------------
    // Live test
    // ---------------------------------------------------------------------

    /**
     * Runs one iteration of the live test using the default candidate
     * ("no overshoot" for position mode, "classic ZN" for velocity mode),
     * applying motor power. Call only while {@code isTuningSuccessful()}.
     *
     * @param now monotonically increasing time in seconds (e.g. {@code getRuntime()})
     * @return telemetry lines describing the live-test state
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
        double output = liveController.calculate(setpoint, measurement, now);
        motor.setPower(output);

        lines.add("=== LIVE TEST ACTIVE ===");
        if (positionMode) {
            lines.add(String.format("Position: %.0f  Setpoint: %.1f  Error: %.1f", measurement, setpoint, setpoint - measurement));
        } else {
            lines.add(String.format("Velocity: %.1f ticks/s%s", measurement, rpmString(measurement)));
            lines.add(String.format("Target:   %.1f ticks/s%s", setpoint, rpmString(setpoint)));
            lines.add(String.format("Error:    %.1f ticks/s%s", setpoint - measurement, rpmString(Math.abs(setpoint - measurement))));
        }
        lines.add(String.format("Output power: %.3f", output));
        return lines;
    }

    /** Stops the live test (if running) and zeroes motor power. Call when the live-test trigger is released. */
    public void stopLiveTest() {
        if (liveTestRunning) {
            motor.setPower(0);
            liveTestRunning = false;
        }
    }

    /** Zeroes motor power. Call at the end of the OpMode. */
    public void stop() {
        motor.setPower(0);
    }
}
