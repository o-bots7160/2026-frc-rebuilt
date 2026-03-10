# Trigger Bindings

This package maps physical controller inputs to robot commands so
`RobotContainer` stays lean. All bindings live in `TriggerBindings`; tunable
parameters (response curves, speed scales, deadbands) live in
`TriggerBindingsConfig`.

## Controllers

The robot uses two Logitech F310 controllers (XInput mode) connected via USB:

| Port | Role     | Primary responsibility                     |
| ---- | -------- | ------------------------------------------ |
| 0    | Driver   | Driving, speed tiers                       |
| 1    | Operator | Gameplay state transitions (fire, harvest) |

## Driver controller layout

![Driver controller layout](../../../../../../../assets/controls-driver.svg)

The driver controller handles field-relative driving and speed management:

- **Left stick** — translation (Y = forward/back, X = strafe).
- **Right stick X** — rotation (omega).
- **Right trigger** — sprint (100 % max speed).
- **Left trigger** — slow/precision mode (40 % max speed).
- No trigger held — normal speed (80 % max speed).

Each axis passes through a configurable response curve
(`leftStickYResponseExponent`, `leftStickXResponseExponent`,
`rightStickXResponseExponent`) and a joystick deadband before the speed scale is
applied.

## Operator controller layout

![Operator controller layout](../../../../../../../assets/controls-operator.svg)

The operator controller manages gameplay state transitions:

| Input         | Action                        |
| ------------- | ----------------------------- |
| Right trigger | Enter **FIRE_READY** state    |
| Left trigger  | Enter **HARVEST_READY** state |
| Start button  | Enter **CLIMB_READY** state   |
| B button      | **EJECT** (active while held) |
| Y button      | Return to **IDLE**            |

Triggers use `onTrue` so the state change persists after the button is released.
The B button uses `whileTrue` because ejecting should stop as soon as the
operator lets go.

## Tuning mode

When `tuningEnabled` is set to `true` in `TriggerBindingsConfig`, the driver
controller switches to subsystem test mode:

- A dashboard chooser (`TriggerBindings/TestSubsystem`) selects which subsystem
  is under test (Shooter, Indexer, Feeder, Intake, Turret, or Harvester).
- **A button** — reverse / move to minimum setpoint.
- **B button** — forward / move to maximum setpoint.
- **X button** — run a full SysId characterization sweep (~60 seconds).

Default commands are not registered in tuning mode so mechanisms stay wherever
the test commands leave them.

## Key classes

| Class                   | Purpose                                             |
| ----------------------- | --------------------------------------------------- |
| `TriggerBindings`       | Wires controller buttons/axes to commands           |
| `TriggerBindingsConfig` | Tunable parameters for response curves, speed tiers |
