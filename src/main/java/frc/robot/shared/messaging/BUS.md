# Robot Message Bus

## Motivation

Before the message bus, subsystem communication was wired through RobotContainer:

```java
aprilTagVisionSubsystem = new AprilTagVisionSubsystem(
        config,
        fieldLayout,
        driveBaseSubsystem::addVisionMeasurement,  // consumer
        driveBaseSubsystem::getPose);              // provider
```

This created several issues:

1. **Bi-directional coupling** - Vision needed both a consumer and provider from DriveBase
2. **RobotContainer as matchmaker** - All subsystem relationships had to be orchestrated in one place
3. **Constructor ordering sensitivity** - DriveBase had to exist before Vision could be constructed
4. **Testing friction** - Hard to test Vision without a real DriveBase instance

## Solution

The `RobotMessageBus` provides a shared communication layer in the `shared.messaging` package. Both subsystems depend only on this package, never on each other.

```
DriveBaseSubsystem ──→ shared.messaging ←── AprilTagVisionSubsystem
```

### How It Works

**DriveBase (in constructor):**
```java
RobotMessageBus.registerPoseSupplier(this::getPose);
RobotMessageBus.subscribeToVisionPose(this::addVisionMeasurement);
```

**Vision (in periodic):**
```java
RobotMessageBus.publishVisionPose(pose, timestamp, stdDevs);

// For simulation, get pose without direct DriveBase reference:
RobotMessageBus.getPoseSupplier()
```

**RobotContainer (simplified):**
```java
driveBaseSubsystem = new DriveBaseSubsystem(config);  // registers itself
visionSubsystem = new AprilTagVisionSubsystem(config, fieldLayout);  // uses bus
```

## Design Decisions

### Static Registry
The bus uses static methods for simplicity. In FRC, there's only one robot instance, so a singleton pattern is appropriate. The `reset()` method supports testing.

### Functional Interfaces
`VisionPoseConsumer` and `RobotPoseSupplier` are functional interfaces that match WPILib's pose estimator API, making integration seamless.

### Ordering Still Matters
DriveBase must be constructed before Vision so the pose supplier is registered before Vision tries to use it in simulation. This is documented in RobotContainer.

## When to Extend

Consider adding new channels to the bus when:
- Two subsystems need to communicate without direct dependency
- A subsystem produces data that multiple consumers might want
- You want to decouple command-level logic from subsystem internals

Examples of future channels:
- `publishShooterReady()` / `subscribeToShooterReady()` for intake coordination
- `publishTargetLocked()` for driver feedback or auto-fire logic
