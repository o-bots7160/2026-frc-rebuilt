# Shared Commands

Abstract command bases and command factory bases that every subsystem mechanism
reuses. Concrete commands in each `subsystems/<mechanism>/commands/` folder
extend these classes rather than raw WPILib `Command`.

## Hierarchy

```
AbstractSubsystemCommand<TSubsystem>
├── AbstractSetAndSeekCommand<TSubsystem>
├── AbstractVelocityCommand<TSubsystem>
├── AbstractIdleVelocityCommand<TSubsystem>
└── SetAndSeekSettleCommand<TSubsystem>

AbstractSubsystemCommandFactory<TSubsystem>
├── AbstractSetAndSeekCommandFactory<TSubsystem>
└── AbstractVelocityCommandFactory<TSubsystem>
```

## Commands

- **`AbstractSubsystemCommand`** — lightweight base that ties a command to a
  single subsystem, auto-registers it as a requirement, and logs "Starting ..."
  on initialize. Subclasses override `onInitialize()` plus standard WPILib
  lifecycle methods.
- **`AbstractSetAndSeekCommand`** — drives a set-and-seek subsystem toward a
  supplied target using a trapezoidal motion profile. Evaluates the target
  supplier on initialize, calls `seekTarget()` each cycle, and finishes when the
  profile is settled. On interruption it schedules a `SetAndSeekSettleCommand`
  so the mechanism decelerates smoothly.
- **`AbstractVelocityCommand`** — drives a velocity subsystem toward a target
  RPM. Evaluates the RPM supplier on initialize, calls `seekVelocity()` each
  cycle, and finishes when the subsystem reaches target velocity.
- **`AbstractIdleVelocityCommand`** — default command that continuously
  maintains the configured idle RPM for a velocity subsystem. Runs until
  interrupted and stops the motor on non-interrupted end.
- **`SetAndSeekSettleCommand`** — recovery command scheduled after a
  set-and-seek command is interrupted. Retargets the mechanism to its current
  position and runs the profiled controller until settled, then stops the motor.

## Factories

- **`AbstractSubsystemCommandFactory`** — base factory that holds a reference to
  its subsystem. Concrete factories extend this to add mechanism-specific
  builder methods.
- **`AbstractSetAndSeekCommandFactory`** — adds SysId characterization commands
  (`createSysIdQuasistaticCommand`, `createSysIdDynamicCommand`) and an
  encoder-reset command (`createResetEncoderCommand`).
- **`AbstractVelocityCommandFactory`** — adds SysId characterization commands
  and an idle-command builder for velocity mechanisms.

## Code structure

| File                                    | Role                                |
| --------------------------------------- | ----------------------------------- |
| `AbstractSubsystemCommand.java`         | Base command with subsystem logging |
| `AbstractSetAndSeekCommand.java`        | Profiled set-and-seek command       |
| `AbstractVelocityCommand.java`          | Velocity-targeting command          |
| `AbstractIdleVelocityCommand.java`      | Default idle-RPM command            |
| `SetAndSeekSettleCommand.java`          | Post-interruption settle command    |
| `AbstractSubsystemCommandFactory.java`  | Base command factory                |
| `AbstractSetAndSeekCommandFactory.java` | Set-and-seek factory with SysId     |
| `AbstractVelocityCommandFactory.java`   | Velocity factory with SysId         |
