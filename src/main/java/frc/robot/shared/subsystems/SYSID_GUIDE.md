# SysId tuning guide

This guide walks you through characterizing a motor-driven mechanism using
WPILib's SysId tool and our `SysIdHelper` infrastructure. Follow every step —
especially the gain correction in Step 5 — to get feedforward gains that
actually work on the robot.

For background on feedforward, PID, and related terms, see the
[project glossary](../../GLOSSARY.md).

---

## Overview

System Identification (SysId) measures how a mechanism responds to known
voltages and produces three feedforward gains:

- **kS** — the voltage needed to overcome static friction (the "unstick"
  voltage).
- **kV** — the voltage needed per unit of speed to maintain constant velocity.
- **kA** — the voltage needed per unit of acceleration to change speed.

These gains let the feedforward controller predict the right voltage for any
target speed, so the PID controller only has to make small corrections.

## Prerequisites

Before running SysId on a mechanism:

1. The subsystem extends `AbstractMotorSubsystem` (either through
   `AbstractVelocitySubsystem` or `AbstractSetAndSeekSubsystem`).
2. The command factory extends `AbstractVelocityCommandFactory` or
   `AbstractSetAndSeekCommandFactory`, which provide SysId commands
   automatically.
3. The mechanism's motor config has correct values for:
   - `motorRotationsPerMechanismRotation` (gear ratio)
   - `motorInverted`
   - `smartCurrentLimitAmps`
4. SysId buttons are bound in `TriggerBindings` (already wired for all
   subsystems).
5. The mechanism is physically safe to run freely in both directions. Remove
   game pieces, clear the area, and hold the robot securely.

## Step 1: Configure SysId parameters

In `subsystems.json`, set two values in the mechanism's config section:

```json
"sysIdRampRateVoltsPerSecond": 0.25,
"sysIdStepVoltage": 7.0
```

- **Ramp rate** controls how slowly voltage increases during the quasistatic
  (slow ramp) test. Lower values (0.15–0.30) give more data points and a better
  fit, but take longer. Start with `0.25`.
- **Step voltage** is the constant voltage applied during the dynamic (sudden
  step) test. It should be high enough to get the mechanism moving quickly but
  not so high that it hits a hard stop or current limit. `7.0` is a safe default
  for most mechanisms.

## Step 2: Deploy and connect

1. Deploy the code to the robot: `./gradlew deploy`.
2. Open AdvantageScope and connect to the robot at `10.71.60.2`.
3. Enable the robot in teleop mode from the Driver Station.
4. Verify the mechanism responds to normal commands before starting SysId.

## Step 3: Run the four SysId tests

SysId requires four test runs. Use the controller buttons bound in
`TriggerBindings`:

| Test                | What it does                                           |
| ------------------- | ------------------------------------------------------ |
| Quasistatic forward | Slowly ramps voltage from 0 upward                     |
| Quasistatic reverse | Slowly ramps voltage from 0 downward                   |
| Dynamic forward     | Applies a sudden step voltage in the forward direction |
| Dynamic reverse     | Applies a sudden step voltage in the reverse direction |

For each test:

1. Press the button to start the test.
2. Let it run until the mechanism reaches a steady speed (quasistatic) or for
   2–3 seconds (dynamic). The quasistatic tests may need 20–40 seconds for slow
   ramp rates.
3. Press the button again or switch to a different command to stop.
4. Wait 2–3 seconds between tests so the mechanism stops completely.

After all four tests, disable the robot. The SysId data is recorded in the
WPILog file.

## Step 4: Analyze in the SysId tool

1. Open the WPILib SysId analysis tool (search "SysId" in the WPILib command
   palette or VS Code).
2. Load the WPILog file from AdvantageScope or the robot's log directory.
3. Select the correct motor label (it matches the `motorLabel` parameter in
   `SysIdHelper`).
4. Choose "Simple" mechanism type.
5. The tool reports **kS**, **kV**, and **kA**.

Write down all three values before proceeding to the next step.

## Step 5: Correct the gains (critical!)

> **⚠️ This step is mandatory. Skipping it will cause your feedforward to
> overshoot by 6.28x and your mechanism will not track its target.**

WPILib's `SysIdRoutineLog.angularVelocity()` internally converts the velocity
from radians per second to **rotations per second** (dividing by 2π) before
writing it to the log file. The SysId analysis tool reads those rotations/s
values but treats them as if they were radians/s. This inflates every
velocity-dependent gain by a factor of 2π ≈ 6.2832.

Apply this correction to the gains reported by the SysId tool:

| Gain | Raw SysId value | Corrected value   | Why                                  |
| ---- | --------------- | ----------------- | ------------------------------------ |
| kS   | Use as-is       | Use as-is         | No velocity dependency               |
| kV   | Divide by 2π    | `kV_raw / 6.2832` | Proportional to velocity             |
| kA   | Divide by 2π    | `kA_raw / 6.2832` | Proportional to acceleration (dv/dt) |

### Worked example (intake)

The SysId tool reported for the intake:

| Gain | SysId reported | ÷ 2π     | Entered in config |
| ---- | -------------- | -------- | ----------------- |
| kS   | 0.30559        | —        | 0.30559           |
| kV   | 0.52129        | 0.08296  | 0.08296           |
| kA   | 0.056569       | 0.009004 | 0.009004          |

### How to verify the correction is right

Use this quick sanity check after correcting kV:

```
Max achievable RPM ≈ (12V − kS) / kV / (2π / 60)
```

For the intake: `(12 − 0.306) / 0.083 / 0.1047 ≈ 1345 RPM`

If the motor is a NEO Vortex (6784 RPM free speed) with a 4:1 gear ratio, the
theoretical max mechanism speed is `6784 / 4 = 1696 RPM`. Getting ~1345 RPM
under load is realistic. If your corrected kV gives a max speed much higher than
the theoretical max, something is wrong.

## Step 6: Enter gains in config

Update the mechanism's section in `subsystems.json`:

```json
"kS": 0.30559,
"kV": 0.08296,
"kA": 0.009004,
"kP": 0.01,
"kI": 0.0,
"kD": 0.0
```

Start kP at `0.01` as a conservative value. You will tune it on the robot later
(see Step 8).

Also update the velocity limits now that you know the real max speed:

```json
"maximumVelocityRpm": 1300,
"maximumAccelerationRpmPerSecond": 500
```

Set `maximumVelocityRpm` to roughly 80–90% of the calculated max to leave
headroom for the PID controller to make corrections.

## Step 7: Deploy and smoke test

1. Deploy the updated config: `./gradlew deploy`.
2. Enable teleop and command the mechanism to a moderate target (e.g., 400 RPM
   for a roller, 2000 RPM for a flywheel).
3. In AdvantageScope, check:
   - `measuredRpm` tracks close to `targetRpm`.
   - `voltageCommandVolts` is reasonable (not saturated at 12V).
   - The mechanism reaches the target within a few seconds.

If the mechanism barely moves, your kV is likely still too high (you may have
forgotten the ÷2π correction). If it overshoots wildly, double-check kS and
verify the motor direction.

## Step 8: Tune kP

Once feedforward is correct, tune kP to tighten the response:

1. Start with kP = `0.01`.
2. Command the mechanism to a target RPM and watch the error in AdvantageScope.
3. If the steady-state error is too large, double kP.
4. If the mechanism oscillates around the target, halve kP.
5. Repeat until the error is within your `velocityToleranceRpm`.

Leave kI at `0.0` unless there is a persistent steady-state error that kP cannot
fix. Leave kD at `0.0` for velocity mechanisms — derivative gain on a noisy
velocity signal usually does more harm than good.

## Diagnostic telemetry

`SysIdHelper` logs the following values to AdvantageKit under the `SysIdHelper/`
prefix during SysId tests:

| Key                      | What it shows                                    |
| ------------------------ | ------------------------------------------------ |
| `sysIdVelocityRadPerSec` | Mechanism velocity in rad/s (what we compute)    |
| `sysIdVelocityRpm`       | Mechanism velocity in RPM (converted from rad/s) |
| `sysIdRawMotorRpm`       | Raw motor shaft RPM before gear ratio conversion |
| `sysIdPositionRad`       | Mechanism position in radians                    |
| `sysIdVoltage`           | Applied voltage in volts                         |

Use these to verify the SysId routine is seeing correct values. If
`sysIdVelocityRpm` ÷ gear ratio ≈ `sysIdRawMotorRpm`, the conversion chain is
correct.

## Troubleshooting

### kV is suspiciously high (> 0.3 for a fast mechanism)

You probably forgot the ÷2π correction. Divide kV and kA by 6.2832 and redeploy.

### Max calculated RPM is way above the motor's theoretical max

Your kV is too low. Re-run SysId and make sure the mechanism ran freely during
the quasistatic test (no binding, no game pieces jamming the rollers).

### Mechanism runs fine during SysId but not during normal operation

Check that the motor inversion (`motorInverted`) matches for both SysId and
normal commands. Also verify that the gear ratio is the same value in both the
motor config and the SysId conversion chain.

### SysId fit is poor (R² < 0.9)

- Lower the ramp rate to get more data points in the quasistatic test.
- Make sure the mechanism is not hitting a current limit during SysId. Check
  `SupplyCurrentAmps` in AdvantageScope. If it is hitting the limit, raise
  `smartCurrentLimitAmps` temporarily during SysId, then set it back afterward.
- Ensure the mechanism moves freely in both directions. A one-direction bind
  will corrupt the data.

## Quick reference card

```
1. Set sysIdRampRateVoltsPerSecond and sysIdStepVoltage in subsystems.json.
2. Deploy. Run all four SysId tests.
3. Open the SysId tool and read kS, kV, kA.
4. CORRECT: kV = kV_raw / (2 * pi),  kA = kA_raw / (2 * pi),  kS = kS_raw.
5. Enter corrected gains + conservative kP in subsystems.json.
6. Deploy. Verify target tracking in AdvantageScope.
7. Tune kP until error is within tolerance.
```
