package frc.robot.subsystems.turret.commands;

import java.util.function.DoubleUnaryOperator;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.shared.commands.AbstractSetAndSeekCommandFactory;
import frc.robot.shared.targeting.ShootOnTheMoveCalculator;
import frc.robot.subsystems.robotpose.RobotPoseSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.turret.config.TurretSubsystemConfig;

/**
 * Generates commands that operate on the turret subsystem so RobotContainer can stay focused on wiring.
 */
public class TurretSubsystemCommandFactory extends AbstractSetAndSeekCommandFactory<TurretSubsystem> {

    /** SOTM solver instance shared across all tracking commands created by this factory. */
    private ShootOnTheMoveCalculator sotmCalculator;

    /** Reference to the most recently created tracking command, used to read compensated distance. */
    private TrackFieldTargetCommand  lastTrackingCommand;

    /**
     * Creates a factory for commands that share the given turret subsystem instance.
     *
     * @param subsystem turret subsystem instance that commands created by this factory will control
     */
    public TurretSubsystemCommandFactory(TurretSubsystem subsystem) {
        super(subsystem);
    }

    /**
     * Builds a profiled move command that reads its target angle from a supplier.
     *
     * @param targetDegreesSupplier supplier providing the desired turret angle in degrees
     * @return command that drives the turret toward the supplied target
     */
    public MoveTurretToAngleCommand createMoveToAngleCommand(Supplier<Double> targetDegreesSupplier) {
        return new MoveTurretToAngleCommand(subsystem, targetDegreesSupplier);
    }

    /**
     * Builds a profiled move command that drives the turret to a fixed angle.
     *
     * @param targetDegrees desired turret angle in degrees
     * @return command that drives the turret to the fixed target
     */
    public MoveTurretToAngleCommand createMoveToAngleCommand(double targetDegrees) {
        return createMoveToAngleCommand(() -> targetDegrees);
    }

    /**
     * Builds and sets the default turret tracking command with shoot-on-the-move compensation.
     *
     * @param robotPoseSubsystem                   robot pose subsystem providing the fused pose estimate
     * @param targetFieldPositionSupplier          supplier of the target position in field coordinates (meters)
     * @param robotYawRateRadiansPerSecondSupplier supplier of the robot's yaw rate in radians per second (positive is counter-clockwise)
     * @param fieldVelocitySupplier                supplier of the robot's field-relative velocity for SOTM compensation
     * @param tofLookup                            function returning estimated time of flight given a distance in meters
     * @return command that is also set as the turret default
     */
    public Command setDefaultTrackFieldTargetCommand(
            RobotPoseSubsystem robotPoseSubsystem,
            Supplier<Translation2d> targetFieldPositionSupplier,
            Supplier<Double> robotYawRateRadiansPerSecondSupplier,
            Supplier<ChassisSpeeds> fieldVelocitySupplier,
            DoubleUnaryOperator tofLookup) {
        Command command = createTrackFieldTargetCommand(
                robotPoseSubsystem,
                targetFieldPositionSupplier,
                robotYawRateRadiansPerSecondSupplier,
                fieldVelocitySupplier,
                tofLookup);
        subsystem.setDefaultCommand(command);
        return command;
    }

    /**
     * Returns the most recent SOTM-compensated distance from the active tracking command.
     * <p>
     * Use this as the distance supplier for shooter RPM lookup so the flywheel speed matches the effective distance after accounting for robot
     * motion. Falls back to 0.0 if no tracking command has been created yet.
     * </p>
     *
     * @return compensated distance in meters
     */
    public double getCompensatedDistanceMeters() {
        return lastTrackingCommand != null ? lastTrackingCommand.getCompensatedDistanceMeters() : 0.0;
    }

    /**
     * Builds a non-finishing command that profiles the turret to 0 degrees and then holds it there indefinitely.
     * <p>
     * Intended for use with {@code toggleOnTrue} so the operator can lock the turret when vision tracking is inaccurate or offline, then press the
     * same button again to resume the default tracking command.
     * </p>
     *
     * @return non-finishing command that locks the turret at 0 degrees until cancelled
     */
    public Command createLockToZeroCommand() {
        return createMoveToAngleCommand(0.0)
                .andThen(Commands.run(subsystem::seekTarget, subsystem))
                .withName("Lock Turret to Zero");
    }

    /**
     * Builds a command that continuously tracks a field-relative target with shoot-on-the-move compensation.
     *
     * @param robotPoseSubsystem                   robot pose subsystem providing the fused pose estimate
     * @param targetFieldPositionSupplier          supplier of the target position in field coordinates (meters)
     * @param robotYawRateRadiansPerSecondSupplier supplier of the robot's yaw rate in radians per second (positive is counter-clockwise)
     * @param fieldVelocitySupplier                supplier of the robot's field-relative velocity for SOTM compensation
     * @param tofLookup                            function returning estimated time of flight given a distance in meters
     * @return command that aims the turret at the supplied field target position
     */
    private TrackFieldTargetCommand createTrackFieldTargetCommand(
            RobotPoseSubsystem robotPoseSubsystem,
            Supplier<Translation2d> targetFieldPositionSupplier,
            Supplier<Double> robotYawRateRadiansPerSecondSupplier,
            Supplier<ChassisSpeeds> fieldVelocitySupplier,
            DoubleUnaryOperator tofLookup) {
        TurretSubsystemConfig config = subsystem.getConfig();
        sotmCalculator = new ShootOnTheMoveCalculator(
                config.getSotmDragCoefficient(),
                config.getSotmMinSpeedMetersPerSecond(),
                config.getSotmMaxIterations(),
                config.getSotmConvergenceToleranceSeconds());

        // When SOTM is disabled, supply zero velocity so the solver returns the raw target.
        Supplier<ChassisSpeeds> effectiveVelocity = config.isSotmEnabled()
                ? fieldVelocitySupplier
                : () -> new ChassisSpeeds();

        lastTrackingCommand = new TrackFieldTargetCommand(
                subsystem,
                robotPoseSubsystem,
                targetFieldPositionSupplier,
                robotYawRateRadiansPerSecondSupplier,
                effectiveVelocity,
                tofLookup,
                sotmCalculator);
        return lastTrackingCommand;
    }
}
