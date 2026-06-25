package com.aaravdhawan25.pidautotuner.ftc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Auto-tunes PIDF gains for a dual-motor velocity-controlled mechanism
 * (e.g. a two-flywheel shooter, dual intake rollers) using the relay-feedback
 * method. Both motors are driven with the same output and tuned together --
 * the resulting gains are applied identically to both.
 *
 * <p>All logic lives in {@link DualMotorPIDMaster}. This OpMode and
 * {@link TuningConfig} are the only files you need to copy into your TeamCode.
 *
 * <h2>Setup</h2>
 * <ol>
 *     <li>In {@link TuningConfig}, set:
 *         <ul>
 *             <li>{@code MOTOR_NAME} and {@code REVERSED} for the first motor</li>
 *             <li>{@code MOTOR_NAME_2} and {@code REVERSED_2} for the second motor</li>
 *             <li>{@code VELOCITY_TARGET_TICKS_PER_SEC} to your target flywheel speed</li>
 *         </ul>
 *     </li>
 *     <li>Run this OpMode. Press start.</li>
 *     <li><b>Phase 1 (relay test):</b> both motors oscillate around the target
 *         velocity simultaneously. Telemetry shows each motor's velocity and
 *         the average, which is what the tuner uses as its measurement.</li>
 *     <li><b>Phase 2 (feedforward sweep):</b> both motors run at fixed powers
 *         to compute kF.</li>
 *     <li>Read the candidate gain sets from telemetry -- they apply to BOTH motors.</li>
 *     <li>Hold gamepad1.a to live-test the "classic ZN" candidate on both motors.</li>
 * </ol>
 */
@TeleOp(name = "PIDF Auto Tuner (Dual Velocity)")
public class DualMotorVelocityPIDFTunerOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        DualMotorPIDMaster pid = new DualMotorPIDMaster(
                hardwareMap,
                TuningConfig.MOTOR_NAME,   TuningConfig.REVERSED,
                TuningConfig.MOTOR_NAME_2, TuningConfig.REVERSED_2,
                TuningConfig.VELOCITY_TARGET_TICKS_PER_SEC,
                TuningConfig.VELOCITY_HYSTERESIS_TICKS_PER_SEC,
                TuningConfig.RELAY_AMPLITUDE,
                TuningConfig.CYCLES_TO_COLLECT,
                TuningConfig.CYCLES_TO_IGNORE,
                TuningConfig.RELAY_TEST_TIMEOUT_S,
                TuningConfig.FEEDFORWARD_TEST_POWERS,
                TuningConfig.FEEDFORWARD_SETTLE_TIME_S,
                TuningConfig.TUNE_INTEGRAL_TERM,
                TuningConfig.DUAL_ENCODERS);

        telemetry.addLine("=== PIDF Auto Tuner (Dual Velocity) ===");
        telemetry.addData("Motor 1", TuningConfig.MOTOR_NAME + (TuningConfig.REVERSED ? " (reversed)" : ""));
        telemetry.addData("Motor 2", TuningConfig.MOTOR_NAME_2 + (TuningConfig.REVERSED_2 ? " (reversed)" : ""));
        telemetry.addData("Encoder mode", TuningConfig.DUAL_ENCODERS ? "DUAL (both motors)" : "SINGLE (motor 1 only)");
        telemetry.addData("Target velocity (ticks/s)", TuningConfig.VELOCITY_TARGET_TICKS_PER_SEC);
        telemetry.addLine("Press START. Both motors will oscillate automatically.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Phase 1 (relay test) + Phase 2 (feedforward sweep) -- automatic
        while (opModeIsActive() && !pid.isTuningComplete()) {
            pid.tuningStep(getRuntime());
            telemetry.clearAll();
            for (String line : pid.getTelemetryLines()) telemetry.addLine(line);
            telemetry.update();
        }

        if (pid.timedOut() || !pid.isTuningSuccessful()) {
            while (opModeIsActive()) {
                telemetry.clearAll();
                for (String line : pid.getTelemetryLines()) telemetry.addLine(line);
                telemetry.update();
            }
            pid.stop();
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

        pid.stop();
    }
}
