package org.firstinspires.ftc.teamcode;

import com.aaravdhawan25.pidautotuner.ftc.PIDMaster;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Auto-tunes PID gains for a run-to-position mechanism (arm, lift, turret,
 * etc.) using the relay-feedback method.
 *
 * <p>All of the actual logic (relay test, candidate computation, live test)
 * lives in {@link PIDMaster}, which comes from the {@code pidautotuner-ftc}
 * JitPack dependency. This OpMode and {@link TuningConfig} are the only two
 * files you need to copy into your own TeamCode -- everything else is
 * resolved via the library dependency.
 *
 * <h2>Setup</h2>
 * <ol>
 *     <li>Edit {@link TuningConfig} -- set {@code MOTOR_NAME},
 *         {@code POSITION_TARGET_TICKS}, and check {@code REVERSED}.</li>
 *     <li>Make sure the mechanism can safely move
 *         {@code +/- POSITION_TARGET_TICKS} from its current position without
 *         hitting hard stops -- the relay test will oscillate across that
 *         range repeatedly.</li>
 *     <li>Run this OpMode. Press start, then stand back -- the mechanism will
 *         oscillate on its own for a few seconds.</li>
 *     <li>Read the candidate gain sets off the Driver Station telemetry.</li>
 *     <li>Optional: hold gamepad1.a to live-test the "no overshoot" candidate
 *         by holding the mechanism at the target position with a real PID loop.</li>
 * </ol>
 *
 * <h2>Notes</h2>
 * <p>For mechanisms affected by gravity (arms, lifts), the relay test alone
 * will not capture a gravity feedforward term -- you may still need to add a
 * constant gravity feedforward ({@code kF * cos(angle)} or similar) on top of
 * these gains in your final code.
 */
@TeleOp(name = "PID Auto Tuner (Position)")
public class PositionPIDTunerOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        PIDMaster pid = new PIDMaster(
                hardwareMap, TuningConfig.MOTOR_NAME, TuningConfig.REVERSED, true,
                TuningConfig.POSITION_TARGET_TICKS, TuningConfig.POSITION_HYSTERESIS_TICKS,
                TuningConfig.RELAY_AMPLITUDE, TuningConfig.CYCLES_TO_COLLECT, TuningConfig.CYCLES_TO_IGNORE,
                TuningConfig.RELAY_TEST_TIMEOUT_S, null, 0, TuningConfig.TUNE_INTEGRAL_TERM);

        telemetry.addLine("=== PID Auto Tuner (Position) ===");
        telemetry.addData("Motor", TuningConfig.MOTOR_NAME);
        telemetry.addData("Target offset (ticks)", TuningConfig.POSITION_TARGET_TICKS);
        telemetry.addLine("Press START. The mechanism will oscillate on its own.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Phase 1: relay test (runs automatically)
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

        // Phase 2: show results, optionally live-test
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
