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
 *     <li>Edit {@link TuningConfig} -- set {@code MOTOR_NAME},
 *         {@code POSITION_TARGET_TICKS}, and check {@code REVERSED}.</li>
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
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, TuningConfig.MOTOR_NAME);
        motor.setDirection(TuningConfig.REVERSED
                ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("=== PID Auto Tuner (Position) ===");
        telemetry.addData("Motor", TuningConfig.MOTOR_NAME);
        telemetry.addData("Target offset (ticks)", TuningConfig.POSITION_TARGET_TICKS);
        telemetry.addLine("Press START. The mechanism will oscillate on its own.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        int startPosition = motor.getCurrentPosition();
        double setpoint = startPosition + TuningConfig.POSITION_TARGET_TICKS;

        RelayAutoTuner tuner = new RelayAutoTuner(
                setpoint,
                TuningConfig.RELAY_AMPLITUDE,
                TuningConfig.POSITION_HYSTERESIS_TICKS,
                TuningConfig.CYCLES_TO_COLLECT,
                TuningConfig.CYCLES_TO_IGNORE
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
                    TuningConfig.CYCLES_TO_COLLECT + TuningConfig.CYCLES_TO_IGNORE);
            telemetry.addData("Elapsed (s)", "%.1f", now);
            telemetry.update();

            if (now > TuningConfig.RELAY_TEST_TIMEOUT_S) {
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

        List<PIDGains> candidates = ZieglerNicholsCalculator.computeCandidates(result, 0.0);

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
