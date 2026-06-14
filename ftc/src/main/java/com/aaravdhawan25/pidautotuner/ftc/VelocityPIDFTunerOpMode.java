package com.aaravdhawan25.pidautotuner.ftc;

import com.aaravdhawan25.pidautotuner.FeedforwardCharacterizer;
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
 * Auto-tunes PIDF gains for a velocity-controlled mechanism (flywheel,
 * intake, etc.) using the relay-feedback method for P/I/D and an open-loop
 * characterization sweep for the feedforward term F.
 *
 * <h2>Setup</h2>
 * <ol>
 *     <li>Edit {@link TuningConfig} -- set {@code MOTOR_NAME},
 *         {@code VELOCITY_TARGET_TICKS_PER_SEC} to roughly your real shooting
 *         speed, and check {@code REVERSED}.</li>
 *     <li>Run this OpMode. Press start.</li>
 *     <li><b>Phase 1 (relay test):</b> the motor will rapidly switch between
 *         full power and zero/reverse power, oscillating its velocity around
 *         the target. This produces Ku and Tu.</li>
 *     <li><b>Phase 2 (feedforward sweep):</b> the motor runs open-loop at a
 *         few fixed power levels in turn, holding each for
 *         {@code FEEDFORWARD_SETTLE_TIME_S} to measure steady-state velocity.
 *         This produces kF.</li>
 *     <li>Read the candidate PIDF gain sets off the Driver Station telemetry.</li>
 *     <li>Optional: hold gamepad1.a to live-test the "classic ZN" candidate by
 *         spinning the mechanism up to the target velocity with a real PIDF
 *         loop.</li>
 * </ol>
 *
 * <h2>Notes</h2>
 * <p>The relay test here drives the motor with raw power (not the FTC SDK's
 * built-in velocity-PIDF mode), so it works the same regardless of what
 * internal PIDF coefficients the hub currently has configured. The resulting
 * kP/kI/kD/kF can either be used with this library's {@link PIDFController}
 * directly in your OpMode loop, or handed to
 * {@code DcMotorEx.setVelocityPIDFCoefficients(...)} if you prefer the SDK's
 * built-in velocity controller -- note the SDK's internal units may need
 * scaling depending on firmware version.
 */
@TeleOp(name = "PIDF Auto Tuner (Velocity)")
public class VelocityPIDFTunerOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, TuningConfig.MOTOR_NAME);
        motor.setDirection(TuningConfig.REVERSED
                ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("=== PIDF Auto Tuner (Velocity) ===");
        telemetry.addData("Motor", TuningConfig.MOTOR_NAME);
        telemetry.addData("Target velocity (ticks/s)", TuningConfig.VELOCITY_TARGET_TICKS_PER_SEC);
        telemetry.addLine("Press START. Phase 1: relay test (automatic).");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ---------------- Phase 1: relay test for Ku / Tu ----------------

        RelayAutoTuner tuner = new RelayAutoTuner(
                TuningConfig.VELOCITY_TARGET_TICKS_PER_SEC,
                TuningConfig.RELAY_AMPLITUDE,
                TuningConfig.VELOCITY_HYSTERESIS_TICKS_PER_SEC,
                TuningConfig.CYCLES_TO_COLLECT,
                TuningConfig.CYCLES_TO_IGNORE
        );

        ElapsedTime timer = new ElapsedTime();
        boolean timedOut = false;

        while (opModeIsActive() && !tuner.isFinished()) {
            double now = timer.seconds();
            double velocity = motor.getVelocity();
            double output = tuner.update(velocity, now);
            motor.setPower(output);

            telemetry.addLine("=== Phase 1: Relay Test ===");
            telemetry.addData("Velocity (ticks/s)", "%.1f", velocity);
            telemetry.addData("Target (ticks/s)", "%.1f", TuningConfig.VELOCITY_TARGET_TICKS_PER_SEC);
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
        sleep(500);

        if (timedOut) {
            telemetry.clearAll();
            telemetry.addLine("=== Tuning TIMED OUT ===");
            telemetry.addLine("The mechanism's velocity didn't complete enough");
            telemetry.addLine("oscillation cycles. Try increasing RELAY_AMPLITUDE,");
            telemetry.addLine("lowering VELOCITY_TARGET_TICKS_PER_SEC, or check wiring.");
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

        // ---------------- Phase 2: feedforward characterization ----------------

        FeedforwardCharacterizer ff = new FeedforwardCharacterizer();

        for (double power : TuningConfig.FEEDFORWARD_TEST_POWERS) {
            if (!opModeIsActive()) break;

            motor.setPower(power);
            ElapsedTime settleTimer = new ElapsedTime();
            double lastVelocity = 0;

            while (opModeIsActive() && settleTimer.seconds() < TuningConfig.FEEDFORWARD_SETTLE_TIME_S) {
                lastVelocity = motor.getVelocity();
                telemetry.addLine("=== Phase 2: Feedforward Sweep ===");
                telemetry.addData("Testing power", "%.2f", power);
                telemetry.addData("Velocity (ticks/s)", "%.1f", lastVelocity);
                telemetry.addData("Settling", "%.1f / %.1f s",
                        settleTimer.seconds(), TuningConfig.FEEDFORWARD_SETTLE_TIME_S);
                telemetry.update();
            }

            ff.addSample(power, lastVelocity);
        }

        motor.setPower(0);
        sleep(500);

        double kF = ff.computeKf();
        if (Double.isNaN(kF)) {
            kF = 0.0; // fall back to no feedforward if characterization failed
        }

        List<PIDGains> candidates = ZieglerNicholsCalculator.computeCandidates(result, kF);

        // Default to the "classic ZN" candidate for the live test.
        PIDGains liveTestGains = candidates.get(4);
        PIDFController liveController = PIDFController.fromGains(liveTestGains);
        liveController.setOutputBounds(-1.0, 1.0);

        boolean liveTestRunning = false;

        while (opModeIsActive()) {
            telemetry.clearAll();
            telemetry.addLine("=== Relay Test Result ===");
            telemetry.addData("Ku", "%.6f", result.Ku);
            telemetry.addData("Tu (s)", "%.4f", result.Tu);
            telemetry.addLine();
            telemetry.addLine("=== Feedforward ===");
            telemetry.addData("kF", "%.6f", kF);
            telemetry.addData("Samples used", ff.sampleCount());
            telemetry.addLine();
            telemetry.addLine("=== Candidate PIDF Gains (kF applied to all) ===");
            for (PIDGains gains : candidates) {
                telemetry.addLine(gains.toString());
            }
            telemetry.addLine();
            telemetry.addLine("Copy a candidate's kP/kI/kD/kF into your code.");
            telemetry.addLine("'classic ZN' is a good starting point for flywheels.");
            telemetry.addLine();
            telemetry.addLine("Hold A to live-test the 'classic ZN' gains by");
            telemetry.addLine("spinning up to the target velocity.");

            if (gamepad1.a) {
                if (!liveTestRunning) {
                    liveController.reset();
                    timer.reset();
                    liveTestRunning = true;
                }
                double velocity = motor.getVelocity();
                double output = liveController.calculate(
                        TuningConfig.VELOCITY_TARGET_TICKS_PER_SEC, velocity, timer.seconds());
                motor.setPower(output);

                telemetry.addLine();
                telemetry.addLine("=== LIVE TEST ACTIVE ===");
                telemetry.addData("Velocity (ticks/s)", "%.1f", velocity);
                telemetry.addData("Target (ticks/s)", "%.1f", TuningConfig.VELOCITY_TARGET_TICKS_PER_SEC);
                telemetry.addData("Error", "%.1f", TuningConfig.VELOCITY_TARGET_TICKS_PER_SEC - velocity);
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
