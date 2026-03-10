# Shared Logging

Wraps AdvantageKit and SmartDashboard telemetry behind a single `Logger` class
so subsystems and commands log consistently without depending on the static
AdvantageKit API directly.

## Usage

```java
// Obtain a logger scoped to the owning class
Logger log = Logger.getInstance(MySubsystem.class);

// Structured telemetry (primary path — goes to AdvantageKit replay logs)
log.recordOutput("targetDegrees", value);

// Verbose telemetry (skipped when FMS-attached to reduce noise)
log.recordVerboseOutput("debugGain", gain);

// Process IO inputs in periodic()
log.processInputs("Motor", motorInputs);

// Operator-critical values on SmartDashboard/Elastic
log.dashboard("Speed", speedRpm);
```

## When to use each method

| Method                | Destination              | When to use                                             |
| --------------------- | ------------------------ | ------------------------------------------------------- |
| `recordOutput`        | AdvantageKit log file    | All structured telemetry                                |
| `recordVerboseOutput` | AdvantageKit (non-FMS)   | Detailed diagnostics that are too noisy for competition |
| `processInputs`       | AdvantageKit log file    | IO input containers each cycle                          |
| `dashboard`           | SmartDashboard / Elastic | Values drivers need live on the dashboard               |
| `dashboardVerbose`    | SmartDashboard (non-FMS) | Dashboard values only needed in practice                |

## Code structure

| File          | Role                                                                                                        |
| ------------- | ----------------------------------------------------------------------------------------------------------- |
| `Logger.java` | Factory-instantiated logger with class-scoped prefixing, AdvantageKit telemetry, and SmartDashboard helpers |
