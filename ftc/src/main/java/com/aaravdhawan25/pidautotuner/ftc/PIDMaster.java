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
 * Orchestrates the full relay-feedback auto-tuning workflow for a single motor,
 * including the relay test, optional feedforward characterisation sweep, candidate
 * gain computation, and an interactive live-test phase.
 *
 * <p>This class handles both position-control tuning (run-to-position mechanisms
 * such as arms and lifts) and velocity-control tuning (constant-speed mechanisms
 * such as flywheels and intakes). The operating mode is selected via the
 * {@code positionMode} constructor argument.
 *
 * <p>This class resides in the {@code pidautotuner-ftc} library. It is pulled in
 * automatically via the JitPack dependency and does not need to be copied into
 * TeamCode. Only the OpMode wrappers and {@code TuningConfig} are user-editable.
 *
 * <h2>Usage pattern</h2>
 * <pre>{@code
 * PIDMaster pid = new PIDMaster(hardwareMap, TuningConfig.MOTOR_NAME,
 *         TuningConfig.REVERSED, false,
 *         TuningConfig.effectiveTargetTicksPerSec(),
 *         TuningConfig.VELOCITY_HYSTERESIS_TICKS_PER_SEC,
 *         TuningConfig.RELAY_AMPLITUDE,
 *         TuningConfig.CYCLES_TO_COLLECT, TuningConfig.CYCLES_TO_IGNORE,
 *         TuningConfig.RELAY_TEST_TIMEOUT_S,
 *         TuningConfig.FEEDFORWARD_TEST_POWERS,
 *         TuningConfig.FEEDFORWARD_SETTLE_TIME_S,
 *         TuningConfig.TUNE_INTEGRAL_TERM,
 *         TuningConfig.TICKS_PER_REV);
 *
 * // Tuning loop
 * while (opModeIsActive() && !pid.isTuningComplete()) {
 *     pid.tuningStep(getRuntime());
 *     for (String line : pid.getTelemetryLines()) telemetry.addLine(line);
 *     telemetry.update();
 * }
 *
 * // Results and live-test loop
 * while (opModeIsActive()) {
 *     telemetry.clearAll();
 *     for (String line : pid.getResultTelemetryLines()) telemetry.addLine(line);
 *     if (gamepad1.a) pid.liveTestStep(getRuntime()); else pid.stopLiveTest();
 *     telemetry.update();
 * }
 * pid.stop();
 * }</pre>
 */
public class PIDMaster {

    /**
     * Internal state machine phases.
     * Progresses: RELAY_TEST → FEEDFORWARD_SWEEP (velocity only) → RESULTS.
     * Transitions to TIMED_OUT or FAILED on error.
     */
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

    /**
     * The absolute setpoint used by the relay tuner.
     * For position mode: {@code startPosition + targetValue}.
     * For velocity mode: equal to {@code targetValue} directly.
     */
    private final double setpoint;

    private Phase phase = Phase.RELAY_TEST;

    // Feedforward sweep tracking
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
     * Ticks per revolution of the output shaft, used to annotate telemetry
     * with RPM values. Set to 0 to disable RPM display.
     */
    private final double ticksPerRev;

    /**
     * Constructs and initialises a {@code PIDMaster}. The motor is immediately
     * configured for open-loop operation ({@code RUN_WITHOUT_ENCODER}) with its
     * encoder reset. No tuning begins until the first call to
     * {@link #tuningStep(double)}.
     *
     * @param hardwareMap            the OpMode hardware map, used to obtain the motor
     * @param motorName              the hardware configuration name of the motor
     * @param reversed               {@code true} to invert the motor direction
     * @param positionMode           {@code true} for position control (arm/lift);
     *                               {@code false} for velocity control (flywheel/intake)
     * @param targetValue            for position mode: the target offset from the motor's
     *                               starting position in encoder ticks; for velocity mode:
     *                               the target speed in ticks/sec
     * @param hysteresis             relay deadband in the same units as {@code targetValue}
     * @param relayAmplitude         relay output magnitude (0–1)
     * @param cyclesToCollect        oscillation cycles to include in the Ku/Tu average
     * @param cyclesToIgnore         initial settling cycles to discard before averaging
     * @param relayTestTimeoutS      maximum duration of the relay test in seconds;
     *                               if elapsed without sufficient cycles the tuner
     *                               transitions to {@code TIMED_OUT}
     * @param feedforwardTestPowers  open-loop power levels for the kF characterisation
     *                               sweep; pass {@code null} to skip feedforward (position mode)
     * @param feedforwardSettleTimeS time in seconds to hold each feedforward power level
     *                               before recording the steady-state velocity
     * @param tuneIntegralTerm       {@code false} to use PD-only Ziegler–Nichols rules
     *                               with {@code kI = 0} (recommended for velocity loops);
     *                               {@code true} to include integral terms
     * @param ticksPerRev            encoder ticks per output-shaft revolution, used for
     *                               RPM annotation in telemetry; pass 0 to disable
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
        motor.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(positionMode
                ? DcMotor.ZeroPowerBehavior.BRAKE   // Hold position when power is removed.
                : DcMotor.ZeroPowerBehavior.FLOAT);  // Coast for velocity-controlled mechanisms.
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setpoint = positionMode ? motor.getCurrentPosition() + targetValue : targetValue;
        tuner = new RelayAutoTuner(setpoint, relayAmplitude, hysteresis, cyclesToCollect, cyclesToIgnore);
    }

    /**
     * Returns the current sensor measurement.
     *
     * <p>For <b>position mode</b>: the signed encoder position in ticks.
     *
     * <p>For <b>velocity mode</b>: the <em>absolute</em> motor velocity in ticks/sec.
     * {@link Math#abs} is applied because the relay output may briefly reverse
     * the motor, producing a negative velocity reading. Without this correction,
     * the relay tuner and PID controller would observe a large spurious positive
     * error (target − negative_velocity), causing output spikes and instability.
     */
    private double measure() {
        return positionMode ? motor.getCurrentPosition() : Math.abs(motor.getVelocity());
    }

    // -------------------------------------------------------------------------
    // State queries
    // -------------------------------------------------------------------------

    /**
     * Returns {@code true} when the tuning workflow has terminated, either
     * successfully or due to a timeout or failure. Always check
     * {@link #isTuningSuccessful()} before reading results.
     */
    public boolean isTuningComplete() {
        return phase == Phase.RESULTS || phase == Phase.TIMED_OUT || phase == Phase.FAILED;
    }

    /**
     * Returns {@code true} if tuning completed successfully and gain candidates
     * are available via {@link #getResultTelemetryLines()}.
     */
    public boolean isTuningSuccessful() {
        return phase == Phase.RESULTS;
    }

    /**
     * Returns {@code true} if the relay test was terminated by the safety
     * timeout without collecting sufficient oscillation cycles.
     */
    public boolean timedOut() {
        return phase == Phase.TIMED_OUT;
    }

    // -------------------------------------------------------------------------
    // Tuning step
    // -------------------------------------------------------------------------

    /**
     * Advances the tuning state machine by one iteration. Must be called once
     * per control-loop cycle while {@link #isTuningComplete()} returns
     * {@code false}. Applies motor power internally; the caller does not need
     * to call {@code motor.setPower()} separately.
     *
     * @param now the current OpMode runtime in seconds ({@code getRuntime()})
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
                // Absolute value here for the same reason as in measure(): brief
                // direction reversals between power levels must not corrupt the
                // (power, velocity) sample used to compute kF.
                ffLastMeasurement = Math.abs(motor.getVelocity());

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
                break;
        }
    }

    private void finishTuning() {
        kF = !positionMode ? (Double.isNaN(ff.computeKf()) ? 0.0 : ff.computeKf()) : 0.0;
        candidates = ZieglerNicholsCalculator.computeCandidates(result, kF, tuneIntegralTerm);

        // Select the default live-test candidate.
        // Position: index 2 ("no overshoot") — minimises risk of hitting physical limits.
        // Velocity: index 4 ("classic ZN")   — provides adequate bandwidth for flywheels.
        PIDGains liveTestGains = candidates.get(positionMode ? 2 : 4);
        liveController = PIDFController.fromGains(liveTestGains);

        // Position: bidirectional output range allows the controller to push
        // in either direction to reach the target.
        // Velocity: output is clamped to [0, 1] because reversing a flywheel
        // during the live test is undesirable; reducing power to zero is sufficient.
        if (positionMode) {
            liveController.setOutputBounds(-1.0, 1.0);
        } else {
            liveController.setOutputBounds(0.0, 1.0);
        }

        phase = Phase.RESULTS;
    }

    // -------------------------------------------------------------------------
    // Telemetry
    // -------------------------------------------------------------------------

    /** Returns an RPM annotation string if {@code ticksPerRev} is configured. */
    private String rpmString(double ticksPerSec) {
        if (ticksPerRev > 0) {
            return String.format("  (%.1f RPM)", (ticksPerSec / ticksPerRev) * 60.0);
        }
        return "";
    }

    /**
     * Returns telemetry lines describing the current tuning progress.
     * Should be called each iteration while {@link #isTuningComplete()} is
     * {@code false}, and also when the tuner has finished in a non-success state.
     *
     * @return a list of strings suitable for Driver Station telemetry
     */
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
                lines.add("Insufficient oscillation cycles were collected.");
                lines.add("Increase RELAY_AMPLITUDE, reduce the target value,");
                lines.add("or verify motor name and encoder wiring.");
                break;
            case FAILED:
                lines.add("=== Tuning FAILED ===");
                lines.add("Oscillation amplitude was too small to compute a result.");
                lines.add("Increase RELAY_AMPLITUDE and retry.");
                break;
            case RESULTS:
                lines.addAll(getResultTelemetryLines());
                break;
        }
        return lines;
    }

    /**
     * Returns the relay-test result and the full list of candidate gain sets.
     * Only valid after {@link #isTuningSuccessful()} returns {@code true}.
     *
     * @return a list of strings suitable for Driver Station telemetry
     */
    public List<String> getResultTelemetryLines() {
        List<String> lines = new ArrayList<>();
        if (result == null || candidates == null) {
            lines.add("(no result available)");
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
        lines.add("Copy the chosen gains into your subsystem code.");
        lines.add(positionMode
                ? "Hold A to live-test the 'no overshoot' candidate."
                : "Hold A to live-test the 'classic ZN' candidate.");
        return lines;
    }

    // -------------------------------------------------------------------------
    // Live test
    // -------------------------------------------------------------------------

    /**
     * Runs one iteration of the live test using the default candidate gains.
     * Position mode uses the "no overshoot" candidate; velocity mode uses
     * "classic ZN". Call this each loop iteration while the driver holds the
     * live-test trigger.
     *
     * @param now the current OpMode runtime in seconds
     * @return telemetry lines describing the live-test state
     */
    public List<String> liveTestStep(double now) {
        List<String> lines = new ArrayList<>();
        if (liveController == null) {
            lines.add("(live test unavailable — tuning has not completed successfully)");
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
            lines.add(String.format("Position: %.0f  Setpoint: %.1f  Error: %.1f",
                    measurement, setpoint, setpoint - measurement));
        } else {
            lines.add(String.format("Velocity: %.1f ticks/s%s", measurement, rpmString(measurement)));
            lines.add(String.format("Target:   %.1f ticks/s%s", setpoint, rpmString(setpoint)));
            lines.add(String.format("Error:    %.1f ticks/s%s", setpoint - measurement,
                    rpmString(Math.abs(setpoint - measurement))));
        }
        lines.add(String.format("Output power: %.3f", output));
        return lines;
    }

    /**
     * Stops the live test if it is currently running. Call this when the
     * driver releases the live-test trigger.
     */
    public void stopLiveTest() {
        if (liveTestRunning) {
            motor.setPower(0);
            liveTestRunning = false;
        }
    }

    /**
     * Commands the motor to zero power. Call this at the end of the OpMode's
     * {@code runOpMode()} method.
     */
    public void stop() {
        motor.setPower(0);
    }
}
