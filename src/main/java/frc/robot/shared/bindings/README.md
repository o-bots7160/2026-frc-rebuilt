# Trigger Bindings

This package maps physical controller inputs to robot commands so
`RobotContainer` stays lean. Bindings are split into a class hierarchy:

- `AbstractTriggerBindings` — shared infrastructure (controllers, command
  factories, config, controller health monitoring, input utilities).
- `CompetitionTriggerBindings` — production gameplay bindings for matches.
- `TuningTriggerBindings` — subsystem test and SysId characterization bindings.

`RobotContainer` checks `TriggerBindingsConfig.getTuningEnabled()` at startup to
instantiate the appropriate subclass. Tunable parameters (response curves, speed
scales, deadbands) live in `TriggerBindingsConfig`.

## Controllers

The robot uses two Logitech F310 controllers (XInput mode) connected via USB:

| Port | Role     | Primary responsibility                     |
| ---- | -------- | ------------------------------------------ |
| 0    | Driver   | Driving, speed tiers                       |
| 1    | Operator | Gameplay state transitions (fire, harvest) |

## Driver controller layout

![Driver controller layout](../../../../../../../assets/controls-driver.svg)

The driver controller handles field-relative driving and speed management:

| Input         | Action                                     |
| ------------- | ------------------------------------------ |
| A button      | _Unassigned_                               |
| B button      | _Unassigned_                               |
| X button      | Spin 180° (hold to maintain heading lock)  |
| Y button      | Snap to nearest field-facing orientation   |
| Right bumper  | Travel mode                                |
| Left bumper   | Trench travel mode                         |
| Right trigger | Sprint (100 % max speed)                   |
| Left trigger  | Slow/precision mode (40 % max speed)       |
| No trigger    | Normal speed (80 % max speed)              |
| Back button   | Wheel lock (X-formation)                   |
| Start button  | _Unassigned_                               |
| D-pad         | Pathfind to configurable field poses       |
| Left stick    | Translation (Y = forward/back, X = strafe) |
| Right stick X | Rotation (omega)                           |

Each axis passes through a configurable response curve
(`leftStickYResponseExponent`, `leftStickXResponseExponent`,
`rightStickXResponseExponent`) and a joystick deadband before the speed scale is
applied.

## Operator controller layout

![Operator controller layout](../../../../../../../assets/controls-operator.svg)

The operator controller manages gameplay state transitions:

| Input         | Action                                         |
| ------------- | ---------------------------------------------- |
| A button      | **EJECT** (active while held)                  |
| B button      | Toggle **turret lock** to 0°                   |
| X button      | Enter **HARVEST_READY** state                  |
| Y button      | Enter **FIRE_READY** state (active while held) |
| Right bumper  | _Unassigned_                                   |
| Left bumper   | _Unassigned_                                   |
| Right trigger | Enter **TRAVEL** state                         |
| Left trigger  | Enter **TRENCH_TRAVEL** state                  |
| Back button   | Return to **IDLE**                             |
| Start button  | _Unassigned_                                   |
| D-pad up      | **Boost** shooter RPM (while held)             |
| D-pad down    | **Cut** shooter RPM (while held)               |
| D-pad left    | _Unassigned_                                   |
| D-pad right   | _Unassigned_                                   |
| Left stick    | _Unassigned_                                   |
| Right stick   | _Unassigned_                                   |

## Tuning mode

When `tuningEnabled` is set to `true` in `TriggerBindingsConfig`,
`RobotContainer` instantiates `TuningTriggerBindings` instead of
`CompetitionTriggerBindings`. In this mode the driver controller switches to
subsystem test mode:

### Tuning driver controller layout

![Tuning driver controller layout](../../../../../../../assets/controls-tuning-driver.svg)

A dashboard chooser (`TriggerBindings/TestSubsystem`) selects which subsystem is
under test (Shooter, Indexer, Feeder, Intake, Turret, or Harvester).

| Input         | Action                                            |
| ------------- | ------------------------------------------------- |
| A button      | Reverse / move to minimum setpoint                |
| B button      | Forward / move to maximum setpoint                |
| X button      | Run a full SysId characterization sweep (~60 s)   |
| Y button      | Run drive base angle then drive SysId in sequence |
| Right bumper  | _Unassigned_                                      |
| Left bumper   | _Unassigned_                                      |
| Right trigger | _Unassigned_                                      |
| Left trigger  | _Unassigned_                                      |
| Back button   | _Unassigned_                                      |
| Start button  | _Unassigned_                                      |
| D-pad         | _Unassigned_                                      |
| Left stick    | _Unassigned_                                      |
| Right stick   | _Unassigned_                                      |

### Tuning operator controller layout

![Tuning operator controller layout](../../../../../../../assets/controls-tuning-operator.svg)

The operator controller provides per-module drive base characterization:

| Input         | Action                         |
| ------------- | ------------------------------ |
| A button      | SysId: drive motor front-left  |
| B button      | SysId: drive motor front-right |
| X button      | SysId: drive motor back-left   |
| Y button      | SysId: drive motor back-right  |
| Left bumper   | SysId: angle motor front-left  |
| Right bumper  | SysId: angle motor front-right |
| Left trigger  | SysId: angle motor back-left   |
| Right trigger | SysId: angle motor back-right  |
| Back button   | _Unassigned_                   |
| Start button  | _Unassigned_                   |
| D-pad         | _Unassigned_                   |
| Left stick    | _Unassigned_                   |
| Right stick   | _Unassigned_                   |

Default commands are not registered in tuning mode so mechanisms stay wherever
the test commands leave them.

## Controller health monitoring

`AbstractTriggerBindings.checkControllerHealth()` runs once per robot loop
(wired through `RobotContainer.periodic()`). It performs two checks per
controller:

1. **Connection check** — Uses `DriverStation.isJoystickConnected()` and fires a
   `DriverStation.reportWarning()` once when a controller transitions from
   connected to disconnected.
2. **Stale-input detection** — During teleop, tracks whether any axis on the
   controller has been non-zero. If the controller reports as connected but all
   axes have been exactly `0.0` for ~1 second (50 cycles), a warning fires:
   _"Driver controller is connected but has sent no input for ~1 second. USB
   data pipe may be frozen — try pressing buttons or reboot the DS laptop."_
   This catches the failure mode where the Windows USB data pipe freezes during
   autonomous and the DS still shows the controller as present.

The following values are recorded to AdvantageKit for post-match replay:

| Key                                   | Type    | Description                            |
| ------------------------------------- | ------- | -------------------------------------- |
| `TriggerBindings/DriverConnected`     | boolean | DS reports joystick on driver port     |
| `TriggerBindings/OperatorConnected`   | boolean | DS reports joystick on operator port   |
| `TriggerBindings/DriverInputActive`   | boolean | At least one driver axis is non-zero   |
| `TriggerBindings/OperatorInputActive` | boolean | At least one operator axis is non-zero |

## Controller troubleshooting

### Known issue: controllers appear connected but produce no input

During FRC matches, the Driver Station may show controllers as connected
(visible in the USB tab) while no input data actually reaches the robot.
Autonomous works fine because it does not read controller inputs, but teleop is
completely unresponsive. Pressing F1 (rescan) and restarting the Driver Station
application do **not** fix this — only a **full laptop reboot** resolves it.

**Root cause:** Windows USB Selective Suspend. During autonomous (when the DS is
not actively polling controller inputs for driving), Windows power management
suspends the USB data pipe. When teleop begins, the pipe fails to wake. The DS
still shows the device because the USB device descriptor is cached, but the
actual input stream is dead.

This is a well-documented issue on Chief Delphi affecting Logitech F310
controllers and many other USB gamepads.

### Preventing USB sleep — Windows configuration

Perform **all three** of these steps on the Driver Station laptop:

#### 1. Disable USB Selective Suspend in Power Options

1. Open **Control Panel → Power Options**.
2. Click **Change plan settings** on the active plan.
3. Click **Change advanced power settings**.
4. Expand **USB settings → USB selective suspend setting**.
5. Set both **On battery** and **Plugged in** to **Disabled**.
6. Click **OK**.

#### 2. Disable per-device USB power management

1. Open **Device Manager**.
2. Expand **Universal Serial Bus controllers**.
3. For **every** USB Root Hub **and** Generic USB Hub entry: a. Right-click →
   **Properties** → **Power Management** tab. b. **Uncheck** "Allow the computer
   to turn off this device to save power." c. Click **OK**.
4. Repeat for every hub entry — there are usually 3–6.

#### 3. Set the Windows Power Plan to High Performance

1. Open **Settings → System → Power & battery** (or **Power Options**).
2. Set **Power mode** to **Best performance**.
3. Set **Put the computer to sleep** to **Never** for both battery and plugged
   in.
4. Turn off **Energy Saver**.

### Pre-match controller checklist

Before every match, the drive coach or a pit crew member should verify:

1. **Wiggle both sticks** on each controller and confirm the corresponding
   entries light up **green** in the DS USB tab.
2. **Press a few buttons** and confirm the button indicators respond.
3. If a controller appears **grayed out** or does not respond to input,
   **immediately reboot the laptop** — do not rely on F1 (rescan) or restarting
   the DS application.
4. **Plug in the laptop charger** at the player station. Windows is less
   aggressive about USB power management when plugged in.
5. Lock controllers into their USB slots in the DS (drag to the correct port
   number and double-click to lock).

### Hardware tips

- Use a **powered USB hub** to prevent voltage sag from causing intermittent
  disconnects, especially if the laptop also powers the Ethernet adapter via
  USB.
- Prefer **direct USB-A ports** on the laptop over USB-C adapters or docks.
  Adapters add another layer where power management can interfere.
- Use **short, high-quality USB cables** (under 6 feet). Long or damaged cables
  increase the chance of signal degradation and intermittent disconnects.
- Consider using **USB port savers** (short USB-A male-to-female extensions) to
  reduce wear on the laptop's built-in ports.
- Ensure the Logitech F310 back switch is set to **X** (XInput mode), not D
  (DirectInput). XInput provides better driver support on Windows.

### F1 rescan limitations

The F1 key (or the rescan button in the DS USB tab) only rescans the **USB
device list** — it does **not** power-cycle the USB stack or reset the data
pipe. If the device descriptor is cached but the data pipe is frozen, F1 will
show the controller as present but input will remain dead. A full laptop reboot
is the only way to reset the USB host controller and restore the data pipe.

When connected to FMS, the DS does not auto-scan for joystick changes while
enabled. Use F1 to trigger a manual scan if a controller was physically
unplugged and reconnected during a match.

## Key classes

| Class                        | Purpose                                                          |
| ---------------------------- | ---------------------------------------------------------------- |
| `AbstractTriggerBindings`    | Base class: controllers, factories, health monitoring, utilities |
| `CompetitionTriggerBindings` | Production gameplay bindings for driver and operator             |
| `TuningTriggerBindings`      | Subsystem test and SysId characterization bindings               |
| `TriggerBindingsConfig`      | Tunable parameters for response curves, speed tiers              |
| `DriverControllerConfig`     | D-pad pathfinding targets and constraints                        |
| `DpadTargetConfig`           | Single d-pad direction target pose configuration                 |
| `TrenchZoneConfig`           | Trench travel speed and zone boundary configuration              |
