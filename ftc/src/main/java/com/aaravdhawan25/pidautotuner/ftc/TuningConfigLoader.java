package com.aaravdhawan25.pidautotuner.ftc;

import java.lang.reflect.Field;

/**
 * Loads tuning configuration from a class in the <em>consuming</em> project,
 * so that a team only needs to add one file
 * ({@code org.firstinspires.ftc.teamcode.TuningConfig}) to their TeamCode --
 * everything else (algorithms, OpModes) comes from the
 * {@code pidautotuner-ftc} JitPack dependency.
 *
 * <h2>How it works</h2>
 * <p>At runtime, this class looks for a class named exactly
 * {@code org.firstinspires.ftc.teamcode.TuningConfig} via reflection. If
 * found, it reads {@code public static final} fields from it by name. If the
 * class doesn't exist, or a specific field is missing, the built-in
 * {@code DEFAULT_*} constant is used instead -- so the tuner OpModes always
 * work, just with generic defaults until you add your own config.
 *
 * <h2>Adding your own config</h2>
 * <p>Copy the {@code TuningConfig.java} template from the FtcAutoTune repo
 * README into {@code TeamCode/src/main/java/org/firstinspires/ftc/teamcode/TuningConfig.java}
 * (package {@code org.firstinspires.ftc.teamcode}, class name
 * {@code TuningConfig}, exactly). Any fields you define there override the
 * defaults below; you can omit fields you don't need to change.
 */
public final class TuningConfigLoader {

    private TuningConfigLoader() {}

    /** Fully-qualified name of the user-supplied config class. */
    public static final String CONFIG_CLASS_NAME = "org.firstinspires.ftc.teamcode.TuningConfig";

    // ---- Built-in defaults (used when the field above isn't found) --------

    public static final String DEFAULT_MOTOR_NAME = "motor";
    public static final boolean DEFAULT_REVERSED = false;

    public static final double DEFAULT_RELAY_AMPLITUDE = 0.5;
    public static final int DEFAULT_CYCLES_TO_COLLECT = 6;
    public static final int DEFAULT_CYCLES_TO_IGNORE = 2;
    public static final double DEFAULT_RELAY_TEST_TIMEOUT_S = 15.0;

    public static final double DEFAULT_POSITION_TARGET_TICKS = 400;
    public static final double DEFAULT_POSITION_HYSTERESIS_TICKS = 10;

    public static final double DEFAULT_VELOCITY_TARGET_TICKS_PER_SEC = 1500;
    public static final double DEFAULT_VELOCITY_HYSTERESIS_TICKS_PER_SEC = 30;

    public static final double[] DEFAULT_FEEDFORWARD_TEST_POWERS = {0.5, 0.75, 1.0};
    public static final double DEFAULT_FEEDFORWARD_SETTLE_TIME_S = 1.5;

    public static final boolean DEFAULT_TUNE_INTEGRAL_TERM = false;

    // ---- Cached lookup of the user's config class --------------------------

    private static Class<?> userConfigClass;
    private static boolean lookedUp = false;

    private static Class<?> userConfigClass() {
        if (!lookedUp) {
            try {
                userConfigClass = Class.forName(CONFIG_CLASS_NAME);
            } catch (Throwable t) {
                userConfigClass = null;
            }
            lookedUp = true;
        }
        return userConfigClass;
    }

    /** @return true if a {@code TuningConfig} class was found in the consumer's TeamCode. */
    public static boolean userConfigFound() {
        return userConfigClass() != null;
    }

    // ---- Typed field readers ------------------------------------------------

    public static String getString(String fieldName, String defaultValue) {
        Object value = getField(fieldName);
        return (value instanceof String) ? (String) value : defaultValue;
    }

    public static boolean getBoolean(String fieldName, boolean defaultValue) {
        Object value = getField(fieldName);
        return (value instanceof Boolean) ? (Boolean) value : defaultValue;
    }

    public static int getInt(String fieldName, int defaultValue) {
        Object value = getField(fieldName);
        return (value instanceof Number) ? ((Number) value).intValue() : defaultValue;
    }

    public static double getDouble(String fieldName, double defaultValue) {
        Object value = getField(fieldName);
        return (value instanceof Number) ? ((Number) value).doubleValue() : defaultValue;
    }

    public static double[] getDoubleArray(String fieldName, double[] defaultValue) {
        Object value = getField(fieldName);
        return (value instanceof double[]) ? (double[]) value : defaultValue;
    }

    private static Object getField(String fieldName) {
        Class<?> clazz = userConfigClass();
        if (clazz == null) return null;
        try {
            Field field = clazz.getField(fieldName);
            return field.get(null);
        } catch (Throwable t) {
            return null;
        }
    }
}
