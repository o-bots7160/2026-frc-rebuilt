# 2026-frc-rebuilt

Robot software for the Ludington O-Bots 7160, powering our 2026 FIRST Robotics
Competition "REBUILT" season codebase.

## Getting started

1. **Install the
   [WPILib 2026 release](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html)**
   (this bundles Java 17, GradleRIO, and the VS Code extensions) and ensure the
   WPILib command-line tools are on your `PATH`.
2. **Clone the repository** and let WPILib import vendordep JSON files
   automatically when you first open the project.
3. **Build and test locally** with `./gradlew build` to verify the code compiles
   and unit tests (if present) pass. Inside WPILib VS Code you can trigger the
   same Gradle build with **Ctrl+Shift+B** (or **Cmd+Shift+B** on macOS) or run
   **“WPILib: Build Robot Code”** from the Command Palette.
4. **Run the simulator** via `./gradlew simulateJava` when you want to iterate
   on subsystems or commands without hardware.
5. **Deploy to the roboRIO** using `./gradlew deploy` (ensure the team number in
   `wpilib_preferences.json` matches 7160 before deploying).

## Deploying robot code

When you're ready to push the latest build to hardware, you have two convenient
options inside WPILib VS Code:

- Open the Command Palette and run **“WPILib: Deploy Robot Code”** (this is also
  listed under the deploy robot commands menu).
- Press **F5** to invoke the same deploy action without opening the palette.

Both drive the GradleRIO deploy target, so you'll get identical behavior to
running `./gradlew deploy` in a terminal.

## Project structure

The notable pieces under `src/main` that extend the stock WPILib template are:

- `deploy/subsystems.json` – declarative configuration that flags which
  mechanisms are available when this build runs.
- `java/frc/robot/BuildConstants.java` – build metadata (git hash, compile time,
  etc.) injected by Gradle for on-robot diagnostics.
- `java/frc/robot/config/` – configuration loader/deserializers that translate
  JSON configuration into strongly typed subsystem settings.
- `java/frc/robot/helpers/Logger.java` – utility wrapper tying AdvantageKit and
  WPILib logging together.
- `java/frc/robot/subsystems/` – abstract subsystem base classes and concrete
  subsystem implementations for the 2026 robot.

![System overview diagram for the 2026 robot](./7160-frc-rebuilt.drawio.svg)

This architecture diagram lives in `7160-frc-rebuilt.drawio.svg`; open and edit
it with [draw.io](https://www.drawio.com/download) to keep the visuals current
as the robot evolves.

## Dependencies

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
