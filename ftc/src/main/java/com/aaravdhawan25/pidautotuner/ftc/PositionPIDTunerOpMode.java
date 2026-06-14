package com.aaravdhawan25.pidautotuner.ftc;

import com.aaravdhawan25.pidautotuner.PIDFController;
import com.aaravdhawan25.pidautotuner.PIDGains;
import com.aaravdhawan25.pidautotuner.RelayAutoTuner;
import com.aaravdhawan25.pidautotuner.RelayTuningResult;
import com.aaravdhawan25.pidautotuner.ZieglerNicholsCalculator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

/**
 * Auto-tunes PID gains for a run-to-position mechanism (arm, lift, turret,
 * etc.) using the relay-feedback method.
 *
 * <h2>Setup</h2>
 * <ol>
 *     <li>Add a {@code TuningConfig.java} class at
 *         {@code org.firstinspires.ftc.teamcode.TuningConfig} in your
 *         TeamCode (see the FtcAutoTune README for the template). Set
 *         {@code MOTOR_NAME}, {@code POSITION_TARGET_TICKS}, and check
 *         {@code REVERSED}. If you don't add this class, built-in defaults
 *         from {@link TuningConfigLoader} are used instead.</li>
 *     <li>Make sure the mechanism can safely move
 *         {@code +/- POSITION_TARGET_TICKS} from its current position without
 *         hitting hard stops -- the relay test will oscillate across that
 *         range repeatedly.</li>
 *     <li>Run this OpMode. Press start, then stand back -- the mechanism will
 *         oscillate on its own for a few seconds.</li>
 *     <li>Read the candidate gain sets off the Driver Station telemetry.</li>
 *     <li>Optional: press gamepad1.a to live-test a candidate by holding the
 *         mechanism at the target position with a real PID loop.</li>
 * </ol>
 *
 * <h2>Notes</h2>
 * <p>For mechanisms affected by gravity (arms, lifts), the relay test alone
 * will not capture a gravity feedforward term -- the "PID (no overshoot)" or
 * "PID (some overshoot)" candidates are usually the best starting points, and
 * you may still need to add a constant gravity feedforward
 * ({@code kF * cos(angle)} or similar) on top of these gains in your final
 * code.
 */
@TeleOp(name = "PID Auto Tuner (Position)")
public class PositionPIDTunerOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        String motorName = TuningConfigLoader.getString("MOTOR_NAME", TuningConfigLoader.DEFAULT_MOTOR_NAME);
        boolean reversed = TuningConfigLoader.getBoolean("REVERSED", TuningConfigLoader.DEFAULT_REVERSED);
        double relayAmplitude = TuningConfigLoader.getDouble("RELAY_AMPLITUDE", TuningConfigLoader.DEFAULT_RELAY_AMPLITUDE);
        int cyclesToCollect = TuningConfigLoader.getInt("CYCLES_TO_COLLECT", TuningConfigLoader.DEFAULT_CYCLES_TO_COLLECT);
        int cyclesToIgnore = TuningConfigLoader.getInt("CYCLES_TO_IGNORE", TuningConfigLoader.DEFAULT_CYCLES_TO_IGNORE);
        double relayTestTimeoutS = TuningConfigLoader.getDouble("RELAY_TEST_TIMEOUT_S", TuningConfigLoader.DEFAULT_RELAY_TEST_TIMEOUT_S);
        double positionTargetTicks = TuningConfigLoader.getDouble("POSITION_TARGET_TICKS", TuningConfigLoader.DEFAULT_POSITION_TARGET_TICKS);
        double positionHysteresisTicks = TuningConfigLoader.getDouble("POSITION_HYSTERESIS_TICKS", TuningConfigLoader.DEFAULT_POSITION_HYSTERESIS_TICKS);

        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, motorName);
        motor.setDirection(reversed
                ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("=== PID Auto Tuner (Position) ===");
        if (!TuningConfigLoader.userConfigFound()) {
            telemetry.addLine("(No org.firstinspires.ftc.teamcode.TuningConfig found");
            telemetry.addLine(" -- using built-in defaults. See FtcAutoTune README.)");
        }
        telemetry.addData("Motor", motorName);
        telemetry.addData("Target offset (ticks)", positionTargetTicks);
        telemetry.addLine("Press START. The mechanism will oscillate on its own.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        int startPosition = motor.getCurrentPosition();
        double setpoint = startPosition + positionTargetTicks;

        RelayAutoTuner tuner = new RelayAutoTuner(
                setpoint,
                relayAmplitude,
                positionHysteresisTicks,
                cyclesToCollect,
                cyclesToIgnore
        );

        ElapsedTime timer = new ElapsedTime();
        boolean timedOut = false;

        while (opModeIsActive() && !tuner.isFinished()) {
            double now = timer.seconds();
            int position = motor.getCurrentPosition();
            double output = tuner.update(position, now);
            motor.setPower(output);

            telemetry.addLine("=== Tuning in progress ===");
            telemetry.addData("Position", position);
            telemetry.addData("Setpoint", "%.1f", setpoint);
            telemetry.addData("Cycles collected", "%d / %d", tuner.cyclesCollected(),
                    cyclesToCollect + cyclesToIgnore);
            telemetry.addData("Elapsed (s)", "%.1f", now);
            telemetry.update();

            if (now > relayTestTimeoutS) {
                timedOut = true;
                break;
            }
        }

        motor.setPower(0);

        if (timedOut) {
            telemetry.clearAll();
            telemetry.addLine("=== Tuning TIMED OUT ===");
            telemetry.addLine("The mechanism didn't complete enough oscillation");
            telemetry.addLine("cycles. Try increasing RELAY_AMPLITUDE, or check");
            telemetry.addLine("that the motor/encoder are wired correctly.");
            telemetry.update();
            while (opModeIsActive()) idle();
            return;
        }

        RelayTuningResult result = tuner.computeResult();
        if (result == null) {
            telemetry.clearAll();
            telemetry.addLine("=== Tuning FAILED ===");
            telemetry.addLine("Not enough oscillation amplitude was measured.");
            telemetry.update();
            while (opModeIsActive()) idle();
            return;
        }

        boolean tuneIntegralTerm = TuningConfigLoader.getBoolean("TUNE_INTEGRAL_TERM", TuningConfigLoader.DEFAULT_TUNE_INTEGRAL_TERM);
        List<PIDGains> candidates = ZieglerNicholsCalculator.computeCandidates(result, 0.0, tuneIntegralTerm);

        // Default to the "no overshoot" candidate (index 2) for the live test --
        // safest starting point for arms/lifts.
        PIDGains liveTestGains = candidates.get(2);
        PIDFController liveController = PIDFController.fromGains(liveTestGains);
        liveController.setOutputBounds(-1.0, 1.0);

        boolean liveTestRunning = false;

        while (opModeIsActive()) {
            telemetry.clearAll();
            telemetry.addLine("=== Relay Test Result ===");
            telemetry.addData("Ku", "%.5f", result.Ku);
            telemetry.addData("Tu (s)", "%.4f", result.Tu);
            telemetry.addData("Oscillation amplitude (ticks)", "%.1f", result.amplitude);
            telemetry.addLine();
            telemetry.addLine("=== Candidate PID Gains ===");
            for (PIDGains gains : candidates) {
                telemetry.addLine(gains.toString());
            }
            telemetry.addLine();
            telemetry.addLine("Copy a candidate's kP/kI/kD into your code.");
            telemetry.addLine("'no overshoot' is the safest starting point.");
            telemetry.addLine();
            telemetry.addLine("Hold A to live-test the 'no overshoot' gains");
            telemetry.addLine("by holding the mechanism at the target position.");

            if (gamepad1.a) {
                if (!liveTestRunning) {
                    liveController.reset();
                    timer.reset();
                    liveTestRunning = true;
                }
                int position = motor.getCurrentPosition();
                double output = liveController.calculate(setpoint, position, timer.seconds());
                motor.setPower(output);

                telemetry.addLine();
                telemetry.addLine("=== LIVE TEST ACTIVE ===");
                telemetry.addData("Position", position);
                telemetry.addData("Setpoint", "%.1f", setpoint);
                telemetry.addData("Error", "%.1f", setpoint - position);
                telemetry.addData("Output power", "%.3f", output);
            } else {
                if (liveTestRunning) {
                    motor.setPower(0);
                    liveTestRunning = false;
                }
            }

            telemetry.update();
        }

        motor.setPower(0);
    }
}
