# Field Target Selection

Provides the data model and runtime logic for choosing which field target the
turret should aim at based on the robot's current position and alliance color.

## How it works

1. **`FieldTargetConfig`** defines fixed target positions from the Blue alliance
   perspective (hub, left rally point, right rally point) along with zone
   boundaries. Red positions are mirrored at runtime by flipping the
   x-coordinate.
2. **`FieldTargetSelector`** evaluates the robot's current pose each cycle and
   selects the appropriate target:
   - **Alliance zone** (between the alliance wall and the zone boundary) → aims
     at the Alliance Hub.
   - **Neutral zone** (beyond the boundary toward mid-field) → aims at the
     closer rally point (left or right, based on the robot's y-coordinate
     relative to the field center divider).

## Integration

`FieldTargetSelector` is constructed in `RobotContainer` with:

- The turret subsystem's `FieldTargetConfig` (from deploy JSON).
- A `Supplier<Pose2d>` from the robot pose subsystem.
- A `Supplier<Optional<Alliance>>` from `RobotEnvironment`.

Commands and subsystems access it via
`fieldTargetSelector::getActiveTargetPosition`.

## Code structure

| File                       | Role                                                |
| -------------------------- | --------------------------------------------------- |
| `FieldTargetConfig.java`   | Data model for hub, rally points, and zone bounds   |
| `FieldTargetSelector.java` | Runtime target selection based on pose and alliance |
