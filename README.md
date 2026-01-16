# 2026-frc-rebuilt

Robot software for the Ludington O-Bots 7160, powering our 2026 FIRST Robotics
Competition "REBUILT" season codebase.

## Contents

- [2026-frc-rebuilt](#2026-frc-rebuilt)
  - [Contents](#contents)
  - [Prerequisites](#prerequisites)
  - [Getting started](#getting-started)
  - [Build and test](#build-and-test)
  - [Simulation](#simulation)
  - [Deployment](#deployment)
  - [Development reference](#development-reference)
    - [Project structure](#project-structure)
    - [Dependencies](#dependencies)

## Prerequisites

1. **Install [Git for Windows](https://git-scm.com/download/win)** if you are on
   Windows. This is required to clone the repository and manage code. (macOS
   users already have Git pre-installed.)
2. **Install the
   [WPILib 2026 release](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html)**
   (bundles Java 17, GradleRIO, and the VS Code extensions). Ensure the WPILib
   command-line tools are on your `PATH`.

## Getting started

1. **Clone the repository** (see GitHub's guide on
   [cloning a repository](https://docs.github.com/en/repositories/creating-and-managing-repositories/cloning-a-repository)
   for step-by-step instructions) and let WPILib import vendordep JSON files
   automatically when you first open the project.
2. **Open the repo folder using the WPILib-installed VS Code** (the WPILib build
   of VS Code is required; standard VS Code will not have the WPILib extensions
   and dependencies bundled).
3. **Install the recommended extensions** when VS Code prompts you. If you don’t
   see a “Recommended” view, open the Command Palette (**Ctrl+Shift+P** or
   **Cmd+Shift+P**), run **“Extensions: Show Recommended Extensions”**, and
   install the workspace recommendations (the WPILib VS Code bundle provides the
   FRC extensions needed for builds and simulation).

## Build and test

1. **Build and test locally** to verify the code compiles and unit tests (if
   present) pass:
   - Terminal: run `./gradlew build`.
   - WPILib Command Palette: run **“WPILib: Build Robot Code”**.
   - Keyboard shortcuts: press **Ctrl+Shift+B** (Windows/Linux) or
     **Cmd+Shift+B** (macOS).

## Simulation

1. **Launch the simulator** to exercise subsystems and commands without
   hardware:
   - Terminal: `./gradlew simulateJava`.
   - WPILib Command Palette: **“WPILib: Simulate Robot Code”**.
2. **Use the WPILib simulator UI** to view NetworkTables, control inputs, and
   AdvantageKit logs while iterating.

## Deployment

1. **Connect to the robot** so the roboRIO is reachable:
   - USB tether from the laptop to the roboRIO (fastest for pits).
   - Ethernet from the laptop to the field/test router (wired is preferred for
     reliability).
   - Team Wi‑Fi (ensure you’re on the robot or practice network and have good
     signal).
2. **Deploy using one of these options** (all run the same GradleRIO deploy
   target):
   - Terminal: `./gradlew deploy`.
   - WPILib Command Palette: **“WPILib: Deploy Robot Code”**.
   - Keyboard shortcut: press **F5** in the WPILib-installed VS Code.
3. **After deployment, bring up your tools to drive and monitor telemetry:**
   - Open the FRC Driver Station to enable the robot and map
     joysticks/controllers.
   - Launch Advantage Scope to visualize logs and signals.
   - Open the Elastic dashboard to watch live telemetry and key widgets.

## Development reference

### Project structure

The key folders under `src/main` you will touch most often are:

- `deploy/` – JSON configs and paths the robot loads at startup, including
  `subsystems.json` and swerve module/controller settings.
- `java/frc/robot/BuildConstants.java` – build metadata (git hash, compile time,
  etc.) injected by Gradle for on-robot diagnostics.
- `java/frc/robot/devices/` – reusable device wrappers like controllers.
- `java/frc/robot/shared/` – cross-cutting pieces many mechanisms share:
  - `bindings/` for trigger/input helpers.
  - `commands/` for abstract command bases.
  - `config/` for shared config types/loaders.
  - `logging/` for AdvantageKit and telemetry helpers.
  - `subsystems/` for abstract subsystem bases.
- `java/frc/robot/subsystems/<mechanism>/` – concrete mechanism code, each with
  its own `commands/`, `config/`, `factories/`, and `io/` folders.

![System overview diagram for the 2026 robot](./7160-frc-rebuilt.drawio.svg)

This architecture diagram lives in `7160-frc-rebuilt.drawio.svg`; open and edit
it with [draw.io](https://www.drawio.com/download) to keep the visuals current
as the robot evolves.

### Dependencies

All third-party libraries live in `vendordeps/` and are versioned alongside the
code so WPILib will automatically pull the right artifacts:

- **AdvantageKit** – provides deterministic telemetry logging and replay so we
  can diagnose the robot off-field.
- **WPILib New Commands** – keeps the declarative command-based framework
  separate so we can stay current with WPILib updates.
- **YAGSL (Yet Another Generic Swerve Library)** – manages the swerve drive
  kinematics, control loops, and auto pathing layers for the drivetrain.
  - **CTRE Phoenix 5 & 6** – covers Talon SRX/Victor SPX legacy controllers (v5)
    plus Talon FX/CANcoder/Pigeon2 devices (v6) used throughout the drivetrain
    and mechanisms.
  - **REVLib** – interfaces with Spark MAX motor controllers, NEO brushless
    motors, and REV sensors used on manipulators.
  - **Studica** – adds support for the Studica control interface hardware that
    powers our custom operator panel.
  - **ThriftyLib** – supplies device wrappers for ThriftyBot sensors (such as
    absolute encoders and power modules) integrated into this robot.

These vendordeps are committed so the correct versions ship with every build,
ensuring sim, practice, and competition robots stay in sync.
