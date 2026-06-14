# FtcAutoTune

An external library for FTC that auto-tunes **PID** (position/run-to-position)
and **PIDF** (velocity, e.g. flywheels) gains directly on your robot, using the
**relay feedback (Astrom-Hagglund) method** combined with **Ziegler-Nichols**
tuning rules.

Structured the same way as [Pedro Pathing](https://github.com/Pedro-Pathing/PedroPathing):
a platform-independent `core` module (the math) and an `ftc` module (Android
library with ready-to-run OpModes), published together so a team's `TeamCode`
module can pull it in as a normal Gradle dependency.

## How it works

1. **Relay test.** The target motor is driven with a "relay" output -- full
   power one way when below the target, full power the other way when above
   it (with a small hysteresis band to avoid noise chatter). This forces a
   sustained oscillation.
2. From that oscillation, the tuner measures the **ultimate gain `Ku`** and
   **ultimate period `Tu`**.
3. **Ziegler-Nichols rules** convert `Ku`/`Tu` into several candidate gain
   sets (P-only, PI, "no overshoot" PID, "some overshoot" PID, classic PID,
   and the aggressive Pessen variant), shown side-by-side on telemetry.
4. **For PIDF (velocity) tuning**, an additional open-loop **feedforward
   sweep** runs the motor at a few fixed power levels, measures steady-state
   velocity at each, and fits `kF` as the slope of `power = kF * velocity`.

References: Ziegler & Nichols (1942), "Optimum Settings for Automatic
Controllers"; Astrom & Hagglund (1984) relay-feedback auto-tuning.

## Project layout

```
FtcAutoTune/
├── core/   - pure Java: RelayAutoTuner, ZieglerNicholsCalculator,
│             PIDFController, FeedforwardCharacterizer, PIDGains, RelayTuningResult
│             (unit tested, no FTC SDK dependency)
├── ftc/    - Android library, FTC SDK compileOnly:
│             PositionPIDTunerOpMode, VelocityPIDFTunerOpMode, TuningConfigLoader
└── templates/
              TuningConfig.java.template - copy this ONE file into your TeamCode
```

## Initial setup

The Gradle wrapper (`gradlew`, `gradlew.bat`, `gradle-wrapper.jar`, pinned to
Gradle 8.9) and `jitpack.yml` (pins the build to JDK 17, required by AGP 8.7)
are committed, so JitPack can build this repo with no extra setup.

If you change `compileSdk` in `ftc/build.gradle.kts`, make sure it stays
**less than or equal to** the `compileSdkVersion` of whatever app consumes
this library (currently 30 in the FtcRobotController quickstart) -- a library
with a higher `compileSdk` than its consumer will fail to build.

## Publishing this library (JitPack), so TeamCode can use it like Pedro Pathing

1. Push this repo to GitHub (e.g. `github.com/aaravdhawan25/FtcAutoTune`).
2. Create a release / tag, e.g. `v0.1.0`. JitPack builds directly from GitHub
   tags -- no manual publishing step needed, as long as the Gradle modules
   have `maven-publish` configured (already done in `core/build.gradle.kts`
   and `ftc/build.gradle.kts`).
3. In your **TeamCode** project (`BB-Decode`), add JitPack to
   `build.dependencies.gradle` (or `TeamCode/build.gradle`) repositories block:

   ```groovy
   repositories {
       maven { url = 'https://jitpack.io' }
   }
   ```

4. Add the dependency:

   ```groovy
   dependencies {
       implementation 'com.github.aaravdhawan25.FtcAutoTune:pidautotuner-core:v0.2.0'
       implementation 'com.github.aaravdhawan25.FtcAutoTune:pidautotuner-ftc:v0.2.0' // OpModes (depends on core)
   }
   ```

   > JitPack publishes each subproject as `com.github.<user>.<repo>:<artifactId>:<tag>`.
   > Run a Gradle sync; the first build will take longer while JitPack builds
   > the tag.

5. Sync Gradle. Two new OpModes will appear on the Driver Station:
   **"PID Auto Tuner (Position)"** and **"PIDF Auto Tuner (Velocity)"** -- they'll
   run immediately using built-in defaults.

6. **(Recommended) Add your own config.** Copy
   [`templates/TuningConfig.java.template`](templates/TuningConfig.java.template)
   to:
   ```
   TeamCode/src/main/java/org/firstinspires/ftc/teamcode/TuningConfig.java
   ```
   (package `org.firstinspires.ftc.teamcode`, class name `TuningConfig`, both
   exactly as shown -- the OpModes find this class via reflection at runtime).
   Edit the fields you care about (`MOTOR_NAME`, `POSITION_TARGET_TICKS` /
   `VELOCITY_TARGET_TICKS_PER_SEC`, etc.) and re-deploy. You can omit any field
   you don't need to change -- it falls back to a built-in default. This is
   the **only file** you need to add; no other FtcAutoTune source goes into
   your TeamCode.

## Using it

### 1. Tune

- In `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/TuningConfig.java`
  (copied from the template, step 6 above), set `MOTOR_NAME` to match your
  hardware configuration name, and set either `POSITION_TARGET_TICKS`
  (position tuner) or `VELOCITY_TARGET_TICKS_PER_SEC` (velocity/PIDF tuner)
  to a realistic value for your mechanism.
- **Safety first:** for arms/lifts, make sure the mechanism can swing
  `+/- POSITION_TARGET_TICKS` around its current position without hitting
  hard stops -- it *will* oscillate through that whole range repeatedly.
  Start with a low `RELAY_AMPLITUDE` (e.g. 0.3) and increase if it doesn't
  oscillate.
- Run the appropriate OpMode. Phase 1 (and Phase 2 for PIDF) run
  automatically -- stand back. If telemetry shows "No ... TuningConfig found",
  the OpMode didn't find your config class (check the package/class name) and
  is using built-in defaults instead.
- Read the candidate gain sets from telemetry. Hold gamepad1 `A` to live-test
  the suggested candidate.

### 2. Use the gains in your own code

Either copy the chosen `kP/kI/kD/kF` into your existing controller, or use
this library's `PIDFController` directly:

```java
PIDFController controller = new PIDFController(kP, kI, kD, kF);
controller.setOutputBounds(-1.0, 1.0);

// in your loop:
double output = controller.calculate(targetVelocity, motor.getVelocity(), timer.seconds());
motor.setPower(output);
```

Call `controller.reset()` whenever the target changes to clear the integral
accumulator and derivative history.

## Which candidate should I pick?

| Candidate | Behavior | Good for |
|---|---|---|
| P only | No overshoot, may have steady-state error | Quick sanity check |
| PI | Removes steady-state error, some lag | Slow mechanisms |
| PID (no overshoot) | Smooth, conservative | Arms/lifts, anything with hard stops |
| PID (some overshoot) | Faster, slight overshoot | Most position mechanisms |
| PID (classic ZN) | Faster still, noticeable overshoot/ringing | Flywheels, fast-settling velocity loops |
| PID (Pessen, aggressive) | Fastest, most overshoot | Use with caution; verify on the live test first |

## Notes & limitations

- The relay test characterizes the *dynamics* around the chosen
  setpoint/velocity -- gains may not generalize perfectly to very different
  setpoints (especially for gravity-affected arms at different angles).
  Re-tune near your actual operating point.
- For arms/lifts, consider adding a separate gravity feedforward term
  (`kF * cos(angle)`) on top of the tuned PID gains in your final code --
  the relay test's `kF` (PIDF tuner only) is a velocity feedforward, not a
  gravity compensation term.
- `RELAY_AMPLITUDE` and the hysteresis values are the main knobs if tuning
  fails (no oscillation, or chattering instead of a clean cycle).
