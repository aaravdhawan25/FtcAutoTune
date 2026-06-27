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
 * this is the main class that runs the whole tuning process for a single motor
 * it handles the relay test, feedforward sweep, computing the gains,
 * and the live test at the end
 *
 * your OpMode just calls tuningStep() every loop and checks isTuningComplete()
 * then calls liveTestStep() when the driver holds A to test the gains
 *
 * works for both position (arms, lifts) and velocity (flywheels, intakes)
 * you just pass positionMode=true or false in the constructor
 *
 * this class is in the library so you dont have to copy it into your teamcode,
 * just add the jitpack dependency and it shows up automatically
 */
public class PIDMaster {

    // the tuning goes through these phases in order
    // RELAY_TEST -> FEEDFORWARD_SWEEP (velocity only) -> RESULTS
    // or RELAY_TEST -> TIMED_OUT/FAILED if something goes wrong
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
    private final double setpoint; // target position or target velocity

    private Phase phase = Phase.RELAY_TEST;

    // feedforward sweep tracking
    private final FeedforwardCharacterizer ff = new FeedforwardCharacterizer();
    private int ffPowerIndex = 0;
    private double ffPhaseStartTime = -1;
    private double ffLastMeasurement = 0;

    // results from the tuning
    private RelayTuningResult result;
    private List<PIDGains> candidates;
    private double kF = 0.0;

    // live test controller
    private PIDFController liveController;
    private boolean liveTestRunning = false;

    // used to show RPM alongside ticks/sec in telemetry (set to 0 to disable)
    private final double ticksPerRev;

    /**
     * sets up the tuner. all your TuningConfig values go here.
     *
     * @param hardwareMap            from the opmode, used to get the motor
     * @param motorName              motor name from hardware config
     * @param reversed               flip direction if your encoder reads backwards
     * @param positionMode           true = arm/lift (position), false = flywheel (velocity)
     * @param targetValue            position: ticks from start. velocity: ticks/sec target
     * @param hysteresis             relay deadband. 10 ticks for position, 30 ticks/sec for velocity
     * @param relayAmplitude         bang-bang power. 0.3-0.7 usually. lower for arms
     * @param cyclesToCollect        how many oscillations to use for the math
     * @param cyclesToIgnore         skip the first few while it warms up
     * @param relayTestTimeoutS      give up after this many seconds if it never oscillates
     * @param feedforwardTestPowers  power levels for kF sweep. null for position mode
     * @param feedforwardSettleTimeS wait this long at each power level before measuring
     * @param tuneIntegralTerm       false = PD only (recommended for flywheels)
     * @param ticksPerRev            your motor's ticks per revolution, for showing RPM. 0 = dont show RPM
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
                ? DcMotor.ZeroPowerBehavior.BRAKE   // brake for position so it holds
                : DcMotor.ZeroPowerBehavior.FLOAT); // float for velocity/flywheels
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (positionMode) {
            // for position mode, target is an offset from where we start
            int startPosition = motor.getCurrentPosition();
            setpoint = startPosition + targetValue;
        } else {
            // for velocity mode, target is just the speed directly
            setpoint = targetValue;
        }

        tuner = new RelayAutoTuner(setpoint, relayAmplitude, hysteresis, cyclesToCollect, cyclesToIgnore);
    }

    /**
     * gets the current measurement from the motor
     *
     * IMPORTANT: for velocity mode we use Math.abs() here
     * without this the velocity goes negative when the relay applies reverse power
     * and then the PID sees a massive error (like target - (-500) = huge number)
     * and it just jitters like crazy. the abs fixes this completely.
     * this was a bug for a while and it drove me crazy lol
     *
     * for position mode we dont abs because negative position is valid
     */
    private double measure() {
        return positionMode ? motor.getCurrentPosition() : Math.abs(motor.getVelocity());
    }

    // -------------------------------------------------------------------------
    // state checks - call these to know whats happening
    // -------------------------------------------------------------------------

    /** true once tuning is done (success OR failure, check isTuningSuccessful() after) */
    public boolean isTuningComplete() {
        return phase == Phase.RESULTS || phase == Phase.TIMED_OUT || phase == Phase.FAILED;
    }

    /** true if tuning worked and you can call getResultTelemetryLines() and liveTestStep() */
    public boolean isTuningSuccessful() {
        return phase == Phase.RESULTS;
    }

    /** true if the relay test ran out of time without oscillating enough */
    public boolean timedOut() {
        return phase == Phase.TIMED_OUT;
    }

    /**
     * call this every loop while !isTuningComplete()
     * it handles the relay test and feedforward sweep automatically
     *
     * @param now current time in seconds, just pass getRuntime()
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
                        // relay test done, now do the feedforward sweep
                        phase = Phase.FEEDFORWARD_SWEEP;
                        ffPowerIndex = 0;
                        ffPhaseStartTime = now;
                    } else {
                        // position mode skips feedforward sweep
                        finishTuning();
                    }
                } else if (now > relayTestTimeoutS) {
                    motor.setPower(0);
                    phase = Phase.TIMED_OUT;
                }
                break;
            }

            case FEEDFORWARD_SWEEP: {
                // run motor at each test power and wait for it to settle
                // then record (power, velocity) to compute kF
                double power = feedforwardTestPowers[ffPowerIndex];
                motor.setPower(power);
                ffLastMeasurement = Math.abs(motor.getVelocity()); // abs here too, same reason as measure()

                if (now - ffPhaseStartTime >= feedforwardSettleTimeS) {
                    ff.addSample(power, ffLastMeasurement);
                    ffPowerIndex++;
                    if (ffPowerIndex >= feedforwardTestPowers.length) {
                        motor.setPower(0);
                        finishTuning();
                    } else {
                        ffPhaseStartTime = now; // move to next power level
                    }
                }
                break;
            }

            case RESULTS:
            case TIMED_OUT:
            case FAILED:
                // nothing to do here, just waiting for the opmode to read the results
                break;
        }
    }

    private void finishTuning() {
        if (!positionMode) {
            double rawKf = ff.computeKf();
            kF = Double.isNaN(rawKf) ? 0.0 : rawKf;
        } else {
            kF = 0.0; // no feedforward for position mode
        }
        candidates = ZieglerNicholsCalculator.computeCandidates(result, kF, tuneIntegralTerm);

        // pick the default candidate for the live test
        // position: index 2 = "no overshoot" (safest for arms)
        // velocity: index 4 = "classic ZN" (works well for flywheels)
        PIDGains liveTestGains = candidates.get(positionMode ? 2 : 4);
        liveController = PIDFController.fromGains(liveTestGains);

        // position mode: full range [-1, 1] since we might need to go both directions
        // velocity mode: [0, 1] only because you never want to reverse a flywheel in live test
        if (positionMode) {
            liveController.setOutputBounds(-1.0, 1.0);
        } else {
            liveController.setOutputBounds(0.0, 1.0);
        }

        phase = Phase.RESULTS;
    }

    // -------------------------------------------------------------------------
    // telemetry stuff
    // -------------------------------------------------------------------------

    /** if ticksPerRev is set, returns " (X RPM)" string to add to velocity displays */
    private String rpmString(double ticksPerSec) {
        if (ticksPerRev > 0) {
            return String.format("  (%.1f RPM)", (ticksPerSec / ticksPerRev) * 60.0);
        }
        return "";
    }

    /** returns lines to show on telemetry while the tuning is running */
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
                lines.add("Didn't oscillate enough.");
                lines.add("Try: increase RELAY_AMPLITUDE, lower target,");
                lines.add("or check motor name and wiring.");
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

    /** returns the final results with all 6 candidate gain sets */
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
        lines.add("Copy a candidate into your code.");
        lines.add(positionMode
                ? "Hold A to live-test 'no overshoot'."
                : "Hold A to live-test 'classic ZN'.");
        return lines;
    }

    // -------------------------------------------------------------------------
    // live test
    // -------------------------------------------------------------------------

    /**
     * runs one iteration of the live test
     * driver holds gamepad A to activate this
     *
     * uses the "no overshoot" candidate for position, "classic ZN" for velocity
     *
     * @param now current time in seconds
     * @return telemetry lines showing whats happening
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

    /** call this when driver releases A to stop the live test */
    public void stopLiveTest() {
        if (liveTestRunning) {
            motor.setPower(0);
            liveTestRunning = false;
        }
    }

    /** stop the motor, call this at the end of the opmode */
    public void stop() {
        motor.setPower(0);
    }
}
