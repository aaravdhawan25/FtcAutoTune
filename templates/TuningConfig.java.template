package org.firstinspires.ftc.teamcode;

/**
 * Edit these values for your robot before running either tuner OpMode.
 * Keeping them in one place makes it easy to re-tune a different mechanism
 * just by changing this file.
 */
public final class TuningConfig {

    private TuningConfig() {}

    // ---- Hardware ------------------------------------------------------

    /** The name of the motor in your hardware configuration, e.g. "flywheel" or "arm". */
    public static final String MOTOR_NAME = "motor";

    /** Set true if positive power should move the mechanism toward a *lower* encoder reading. */
    public static final boolean REVERSED = false;


    /** Set true if running position pid tuning OpMode */

    public static final boolean PositionPIDtuner = false;

    // ---- Relay test tuning ---------------------------------------------

    /**
     * Magnitude of the relay (bang-bang) output, from 0 to 1. Pick the
     * smallest value that reliably overcomes friction/gravity and produces a
     * clean oscillation. 0.3-0.7 is typical. Too high can be violent or
     * unsafe for position mechanisms (arms/lifts) -- start low and increase
     * if the mechanism doesn't oscillate.
     */
    public static final double RELAY_AMPLITUDE = 0.5;

    /** Number of full oscillation cycles to average for the Ku/Tu calculation. */
    public static final int CYCLES_TO_COLLECT = 6;

    /** Number of initial cycles to discard while the oscillation settles. */
    public static final int CYCLES_TO_IGNORE = 2;

    /** Safety timeout for the relay test, in seconds, in case the system never oscillates. */
    public static final double RELAY_TEST_TIMEOUT_S = 15.0;

    // ---- Position tuner specific ----------------------------------------

    /**
     * Target position for the position PID tuner, in encoder ticks relative
     * to the position the motor is at when the OpMode starts.
     */
    public static final double POSITION_TARGET_TICKS = 400;

    /**
     * Hysteresis band for the position relay test, in encoder ticks. The
     * relay switches once the measured position crosses
     * target +/- this value. A few ticks is usually enough; increase if you
     * see rapid chatter instead of a clean oscillation.
     */
    public static final double POSITION_HYSTERESIS_TICKS = 10;

    // ---- Velocity (PIDF) tuner specific -----------------------------------

    /**
     * Target velocity for the velocity PIDF tuner, in encoder ticks per second.
     * Pick a value representative of where you'll actually run the mechanism
     * (e.g. your flywheel's shooting speed).
     */
    public static final double VELOCITY_TARGET_TICKS_PER_SEC = 1500;

    /** Hysteresis band for the velocity relay test, in ticks/sec. */
    public static final double VELOCITY_HYSTERESIS_TICKS_PER_SEC = 30;

    /**
     * Power levels to use for the feedforward (kF) characterization step.
     * The motor is run open-loop at each of these powers in turn, and the
     * steady-state velocity is recorded.
     */
    public static final double[] FEEDFORWARD_TEST_POWERS = {0.5, 0.75, 1.0};

    /** How long to hold each feedforward test power before sampling, in seconds. */
    public static final double FEEDFORWARD_SETTLE_TIME_S = 1.5;

    /**
     * If false, every candidate gain set has {@code kI = 0} and is computed
     * using the Ziegler-Nichols PD (instead of PID) rule family. Common
     * choice for velocity/flywheel loops where {@code kF} already handles
     * steady-state error and an integral term mainly risks windup.
     */
    public static final boolean TUNE_INTEGRAL_TERM = false;

    // ---- Dual motor velocity tuner specific --------------------------------

    /**
     * Hardware config name of the second motor for the dual-motor velocity
     * PIDF tuner (e.g. "flywheel2", "shooterRight"). Both motors will be
     * tuned together and run with the same kP/kI/kD/kF output.
     */
    public static final String MOTOR_NAME_2 = "motor2";

    /**
     * Direction of the second motor. Set true if positive power should run
     * this motor in the opposite physical direction to how the encoder reads
     * (common on shooter/flywheel setups where motors face each other).
     */
    public static final boolean REVERSED_2 = true;

    /**
     * Set to true if BOTH motors have encoders plugged in. Set to false if
     * only the first motor ({@code MOTOR_NAME}) has an encoder connected --
     * in that case the tuner uses only motor 1's velocity as the measurement
     * but still drives both motors with the same output power and gains.
     *
     * <p>This is common on shooters where only one encoder port is used.
     */
    public static final boolean DUAL_ENCODERS = false;
}