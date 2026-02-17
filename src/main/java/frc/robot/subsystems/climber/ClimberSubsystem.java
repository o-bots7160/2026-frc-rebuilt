package frc.robot.subsystems.climber;

import frc.robot.devices.motor.Motor;
import frc.robot.shared.config.RobotEnvironment;
import frc.robot.shared.subsystems.AbstractSetAndSeekSubsystem;
import frc.robot.subsystems.climber.config.ClimberSubsystemConfig;
import frc.robot.subsystems.climber.io.ClimberSensorIO;
import frc.robot.subsystems.climber.io.ClimberSensorIOInputsAutoLogged;
import frc.robot.subsystems.climber.io.ClimberSensorIOReal;
import frc.robot.subsystems.climber.io.ClimberSensorIOSim;

/**
 * Climber subsystem that extends and retracts arms for staged endgame climbs on the tower in REBUILT. Uses two front-facing PlayingWithFusion Time of
 * Flight sensors to detect and align with the tower before climbing.
 * <p>
 * Motor functionality is not yet implemented. The subsystem currently runs with a disabled motor and focuses on Time of Flight sensor telemetry for
 * tower detection and alignment. Once motor hardware is selected, the {@link #buildMotor(ClimberSubsystemConfig)} method should be updated to
 * construct the real motor.
 * </p>
 */
public class ClimberSubsystem extends AbstractSetAndSeekSubsystem<ClimberSubsystemConfig> {

    /**
     * Returns null unconditionally because motor hardware has not been selected yet. The parent class falls back to a {@code DisabledMotor} when it
     * receives null, so all motor commands become safe no-ops.
     *
     * @param config climber configuration bundle (unused until motor hardware is selected)
     * @return null; parent will substitute a {@code DisabledMotor}
     */
    // TODO: implement when motor hardware is selected
    private static Motor buildMotor(ClimberSubsystemConfig config) {
        return null;
    }

    /** Sensor IO implementation for the two front-facing Time of Flight sensors. */
    private final ClimberSensorIO                 sensorIO;

    /** Logged sensor inputs snapshot used for telemetry and replay. */
    private final ClimberSensorIOInputsAutoLogged sensorInputs = new ClimberSensorIOInputsAutoLogged();

    /**
     * Creates the climber subsystem with Time of Flight sensors for tower alignment and a disabled motor placeholder.
     *
     * @param config climber configuration bundle loaded from JSON
     */
    public ClimberSubsystem(ClimberSubsystemConfig config) {
        this(config, buildMotor(config));
    }

    private ClimberSubsystem(ClimberSubsystemConfig config, Motor motor) {
        super(config, motor);

        if (isSubsystemDisabled()) {
            log.verbose("ClimberSubsystem disabled; skipping sensor initialization.");
            this.sensorIO = inputs -> {
            };
            return;
        }

        this.sensorIO = RobotEnvironment.isReal()
                ? new ClimberSensorIOReal(config.climberSensorConfig)
                : new ClimberSensorIOSim();
    }

    @Override
    public void periodic() {
        super.periodic();

        sensorIO.updateInputs(sensorInputs);
        log.processInputs("sensors", sensorInputs);

        // Computed telemetry from sensor readings
        log.recordOutput("towerDetected", isTowerDetected());
        log.recordOutput("alignedToTower", isAlignedToTower());
        log.recordOutput("distanceDifferenceMillimeters", getDistanceDifferenceMillimeters());
    }

    /**
     * Returns the most recent distance reading from the left Time of Flight sensor in millimeters.
     *
     * @return left sensor distance in millimeters, or 0.0 if the subsystem is disabled
     */
    public double getLeftDistanceMillimeters() {
        if (isSubsystemDisabled()) {
            logDisabled("getLeftDistanceMillimeters");
            return 0.0;
        }

        return sensorInputs.leftDistanceMillimeters;
    }

    /**
     * Returns the most recent distance reading from the right Time of Flight sensor in millimeters.
     *
     * @return right sensor distance in millimeters, or 0.0 if the subsystem is disabled
     */
    public double getRightDistanceMillimeters() {
        if (isSubsystemDisabled()) {
            logDisabled("getRightDistanceMillimeters");
            return 0.0;
        }

        return sensorInputs.rightDistanceMillimeters;
    }

    /**
     * Checks whether the robot is aligned to the tower by comparing left and right Time of Flight readings.
     * <p>
     * The robot is considered aligned when both sensors detect the tower (readings below the configured maximum detection distance) and the absolute
     * difference between the two readings is within the configured alignment tolerance.
     * </p>
     *
     * @return true if the robot is centered and square to the tower, false otherwise or if the subsystem is disabled
     */
    public boolean isAlignedToTower() {
        if (isSubsystemDisabled()) {
            logDisabled("isAlignedToTower");
            return false;
        }

        double  maxDistance   = config.climberSensorConfig.getMaximumDetectionDistanceMillimeters();
        double  tolerance     = config.climberSensorConfig.getAlignmentToleranceMillimeters();

        boolean leftDetected  = sensorInputs.leftDistanceMillimeters > 0.0
                && sensorInputs.leftDistanceMillimeters < maxDistance;
        boolean rightDetected = sensorInputs.rightDistanceMillimeters > 0.0
                && sensorInputs.rightDistanceMillimeters < maxDistance;

        if (!leftDetected || !rightDetected) {
            return false;
        }

        return getDistanceDifferenceMillimeters() <= tolerance;
    }

    /**
     * Checks whether either Time of Flight sensor detects the tower within the configured maximum detection distance.
     *
     * @return true if at least one sensor reads below the maximum detection distance, false if the subsystem is disabled
     */
    public boolean isTowerDetected() {
        if (isSubsystemDisabled()) {
            logDisabled("isTowerDetected");
            return false;
        }

        double  maxDistance   = config.climberSensorConfig.getMaximumDetectionDistanceMillimeters();

        boolean leftDetected  = sensorInputs.leftDistanceMillimeters > 0.0
                && sensorInputs.leftDistanceMillimeters < maxDistance;
        boolean rightDetected = sensorInputs.rightDistanceMillimeters > 0.0
                && sensorInputs.rightDistanceMillimeters < maxDistance;

        return leftDetected || rightDetected;
    }

    /**
     * Computes the absolute difference between the left and right Time of Flight sensor readings in millimeters.
     *
     * @return distance difference in millimeters
     */
    private double getDistanceDifferenceMillimeters() {
        return Math.abs(sensorInputs.leftDistanceMillimeters - sensorInputs.rightDistanceMillimeters);
    }
}
