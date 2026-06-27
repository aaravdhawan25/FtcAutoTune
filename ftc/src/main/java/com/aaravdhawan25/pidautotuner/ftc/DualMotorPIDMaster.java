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
 * Orchestrates the relay-feedback velocity auto-tuning workflow for a
 * <em>dual-motor</em> mechanism such as a two-flywheel shooter or a
 * dual-intake roller assembly.
 *
 * <p>Both motors receive identical power output throughout the relay test
 * and feedforward characterisation sweep. The velocity measurement is derived
 * from one or both encoders depending on the {@code dualEncoders} constructor
 * argument:
 * <ul>
 *   <li>{@code dualEncoders = true}: the measurement is the arithmetic mean
 *       of the two motors' absolute velocities, providing a more representative
 *       estimate when both wheels are instrumented.</li>
 *   <li>{@code dualEncoders = false}: the measurement is taken solely from
 *       the first motor's encoder. The second motor is still driven at the
 *       same power as the first, but its encoder is not read. This is the
 *       correct setting when only one encoder port is populated.</li>
 * </ul>
 *
 * <p>The resulting gain set is identical for both motors and is intended to
 * be applied to each independently in the final subsystem code.
 *
 * <p>Each motor has an independent direction setting to accommodate opposing-face
 * flywheel configurations in which one motor must be reversed relative to the other.
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
     * Constructs and initialises a {@code DualMotorPIDMaster}. Both motors are
     * immediately configured for open-loop operation with their encoders reset.
     *
     * @param hardwareMap            the OpMode hardware map
     * @param motorName1             hardware configuration name of the first motor
     * @param reversed1              {@code true} to invert the direction of motor 1
     * @param motorName2             hardware configuration name of the second motor
     * @param reversed2              {@code true} to invert the direction of motor 2;
     *                               typically {@code true} for opposing-face flywheels
     * @param targetVelocity         target speed for both motors in ticks/sec
     * @param hysteresis             relay deadband in ticks/sec
     * @param relayAmplitude         relay output magnitude (0–1)
     * @param cyclesToCollect        oscillation cycles to include in the Ku/Tu average
     * @param cyclesToIgnore         initial settling cycles to discard
     * @param relayTestTimeoutS      relay test safety timeout in seconds
     * @param feedforwardTestPowers  open-loop power levels for kF characterisation
     * @param feedforwardSettleTimeS settle time per power level in seconds
     * @param tuneIntegralTerm       {@code false} to use PD-only rules (recommended
     *                               for velocity loops)
     * @param dualEncoders           {@code true} to average both encoders; {@code false}
     *                               to use only motor 1's encoder (motor 2 is still driven)
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
        motor1.setDirection(reversed1 ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor2 = hardwareMap.get(DcMotorEx.class, motorName2);
        motor2.setDirection(reversed2 ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        tuner = new RelayAutoTuner(targetVelocity, relayAmplitude, hysteresis, cyclesToCollect, cyclesToIgnore);
    }

    /**
     * Returns the velocity measurement used by the relay tuner and controllers.
     *
     * <p>{@link Math#abs} is applied to all encoder readings. Without this,
     * the brief velocity reversal that occurs when the relay switches from
     * positive to negative power would produce a large spurious positive error
     * in the controller, destabilising the feedback loop.
     *
     * @return absolute velocity in ticks/sec, averaged across both encoders
     *         if {@code dualEncoders} is {@code true}, or from motor 1 alone otherwise
     */
    private double measure() {
        if (dualEncoders) {
            return (Math.abs(motor1.getVelocity()) + Math.abs(motor2.getVelocity())) / 2.0;
        } else {
            return Math.abs(motor1.getVelocity());
        }
    }

    /**
     * Applies the same power level to both motors simultaneously.
     *
     * @param power motor power in the range [−1, 1]
     */
    private void setPower(double power) {
        motor1.setPower(power);
        motor2.setPower(power);
    }

    // -------------------------------------------------------------------------
    // State queries
    // -------------------------------------------------------------------------

    /** Returns {@code true} when tuning has terminated (success or failure). */
    public boolean isTuningComplete() {
        return phase == Phase.RESULTS || phase == Phase.TIMED_OUT || phase == Phase.FAILED;
    }

    /** Returns {@code true} if tuning completed successfully. */
    public boolean isTuningSuccessful() {
        return phase == Phase.RESULTS;
    }

    /** Returns {@code true} if the relay test was terminated by the safety timeout. */
    public boolean timedOut() {
        return phase == Phase.TIMED_OUT;
    }

    // -------------------------------------------------------------------------
    // Tuning step
    // -------------------------------------------------------------------------

    /**
     * Advances the tuning state machine by one iteration.
     * Must be called once per control-loop cycle while
     * {@link #isTuningComplete()} returns {@code false}.
     *
     * @param now the current OpMode runtime in seconds
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

        // Velocity default: index 4 ("classic ZN") provides adequate bandwidth
        // for flywheel applications without excessive overshoot.
        PIDGains liveTestGains = candidates.get(4);
        liveController = PIDFController.fromGains(liveTestGains);
        liveController.setOutputBounds(-1.0, 1.0);

        phase = Phase.RESULTS;
    }

    // -------------------------------------------------------------------------
    // Telemetry
    // -------------------------------------------------------------------------

    /**
     * Returns telemetry lines describing the current tuning progress or error state.
     *
     * @return a list of strings suitable for Driver Station telemetry
     */
    public List<String> getTelemetryLines() {
        List<String> lines = new ArrayList<>();
        switch (phase) {
            case RELAY_TEST:
                lines.add("=== Phase 1: Relay Test (Dual Motor) ===");
                lines.add(String.format("Encoder mode: %s", dualEncoders ? "DUAL (averaged)" : "SINGLE (motor 1 only)"));
                lines.add(String.format("Measurement (ticks/s): %.1f", measure()));
                lines.add(String.format("Motor 1 velocity:      %.1f", Math.abs(motor1.getVelocity())));
                if (dualEncoders) {
                    lines.add(String.format("Motor 2 velocity:      %.1f", Math.abs(motor2.getVelocity())));
                } else {
                    lines.add("Motor 2 velocity:      (encoder not connected)");
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
                lines.add("Insufficient oscillation cycles were collected.");
                lines.add("Increase RELAY_AMPLITUDE, reduce the target velocity,");
                lines.add("or verify wiring on both motors.");
                break;
            case FAILED:
                lines.add("=== Tuning FAILED ===");
                lines.add("Oscillation amplitude was too small. Increase RELAY_AMPLITUDE.");
                break;
            case RESULTS:
                lines.addAll(getResultTelemetryLines());
                break;
        }
        return lines;
    }

    /**
     * Returns the relay-test result and candidate gain sets.
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

        lines.add("=== Relay Test Result (Dual Motor) ===");
        lines.add(String.format("Encoder mode: %s", dualEncoders ? "DUAL" : "SINGLE (motor 1 only)"));
        lines.add(String.format("Ku=%.6f  Tu=%.4fs", result.Ku, result.Tu));
        lines.add(String.format("kF=%.6f (samples: %d)", kF, ff.sampleCount()));
        lines.add("");
        lines.add("=== Candidate Gains (apply identically to both motors) ===");
        for (PIDGains gains : candidates) {
            lines.add(gains.toString());
        }
        lines.add("");
        lines.add("Copy the chosen gains into your subsystem code for both motors.");
        lines.add("Hold A to live-test the 'classic ZN' candidate.");
        return lines;
    }

    // -------------------------------------------------------------------------
    // Live test
    // -------------------------------------------------------------------------

    /**
     * Runs one iteration of the live test using the "classic ZN" candidate gains.
     * Both motors are driven with the same computed output.
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

    /**
     * Stops the live test if currently active. Call when the driver releases
     * the live-test trigger.
     */
    public void stopLiveTest() {
        if (liveTestRunning) {
            setPower(0);
            liveTestRunning = false;
        }
    }

    /**
     * Commands both motors to zero power. Call at the end of the OpMode.
     */
    public void stop() {
        setPower(0);
    }
}
