package com.aaravdhawan25.pidautotuner.ftc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Auto-tunes PIDF gains for a velocity-controlled mechanism (flywheel,
 * intake, etc.) using the relay-feedback method for P/I/D and an open-loop
 * characterization sweep for the feedforward term F.
 *
 * <p>All of the actual logic (relay test, feedforward sweep, candidate
 * computation, live test) lives in {@link PIDMaster}, which comes from the
 * {@code pidautotuner-ftc} JitPack dependency. This OpMode and
 * {@link TuningConfig} are the only two files you need to copy into your own
 * TeamCode -- everything else is resolved via the library dependency.
 *
 * <h2>Setup</h2>
 * <ol>
 *     <li>Edit {@link TuningConfig} -- set {@code MOTOR_NAME},
 *         {@code VELOCITY_TARGET_TICKS_PER_SEC} to roughly your real shooting
 *         speed, and check {@code REVERSED}.</li>
 *     <li>Run this OpMode. Press start.</li>
 *     <li><b>Phase 1 (relay test):</b> the motor oscillates its velocity
 *         around the target. This produces Ku and Tu.</li>
 *     <li><b>Phase 2 (feedforward sweep):</b> the motor runs open-loop at the
 *         power levels in {@code FEEDFORWARD_TEST_POWERS}, measuring
 *         steady-state velocity at each. This produces kF.</li>
 *     <li>Read the candidate PIDF gain sets off the Driver Station telemetry.</li>
 *     <li>Optional: hold gamepad1.a to live-test the "classic ZN" candidate by
 *         spinning the mechanism up to the target velocity with a real PIDF loop.</li>
 * </ol>
 */
@TeleOp(name = "FtcAutoTune | Velocity PIDF", group = "FtcAutoTune")
public class VelocityPIDFTunerOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        double targetTicksPerSec = TuningConfig.effectiveTargetTicksPerSec();

        PIDMaster pid = new PIDMaster(
                hardwareMap, TuningConfig.MOTOR_NAME, TuningConfig.REVERSED, false,
                targetTicksPerSec, TuningConfig.VELOCITY_HYSTERESIS_TICKS_PER_SEC,
                TuningConfig.RELAY_AMPLITUDE, TuningConfig.CYCLES_TO_COLLECT, TuningConfig.CYCLES_TO_IGNORE,
                TuningConfig.RELAY_TEST_TIMEOUT_S, TuningConfig.FEEDFORWARD_TEST_POWERS,
                TuningConfig.FEEDFORWARD_SETTLE_TIME_S, TuningConfig.TUNE_INTEGRAL_TERM,
                TuningConfig.TICKS_PER_REV);

        telemetry.addLine("=== PIDF Auto Tuner (Velocity) ===");
        telemetry.addData("Motor", TuningConfig.MOTOR_NAME);
        telemetry.addData("TICKS_PER_REV", TuningConfig.TICKS_PER_REV);
        telemetry.addData("Target (ticks/s)", String.format("%.1f", targetTicksPerSec));
        telemetry.addData("Target (RPM)", String.format("%.1f", TuningConfig.toRPM(targetTicksPerSec)));
        if (TuningConfig.USE_RPM_TARGET) {
            telemetry.addLine("(target specified in RPM, converted to ticks/s)");
        }
        telemetry.addLine("Press START. Phase 1: relay test (automatic).");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Phase 1 (relay test) + Phase 2 (feedforward sweep) run automatically
        while (opModeIsActive() && !pid.isTuningComplete()) {
            pid.tuningStep(getRuntime());
            for (String line : pid.getTelemetryLines()) telemetry.addLine(line);
            telemetry.update();
        }

        if (pid.timedOut() || !pid.isTuningSuccessful()) {
            while (opModeIsActive()) {
                telemetry.clearAll();
                for (String line : pid.getTelemetryLines()) telemetry.addLine(line);
                telemetry.update();
            }
            return;
        }

        // Phase 3: show results, optionally live-test
        while (opModeIsActive()) {
            telemetry.clearAll();
            for (String line : pid.getResultTelemetryLines()) telemetry.addLine(line);

            if (gamepad1.a) {
                telemetry.addLine();
                for (String line : pid.liveTestStep(getRuntime())) telemetry.addLine(line);
            } else {
                pid.stopLiveTest();
            }

            telemetry.update();
        }
    }
}
