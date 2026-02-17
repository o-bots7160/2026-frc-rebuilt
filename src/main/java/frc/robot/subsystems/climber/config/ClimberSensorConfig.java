package frc.robot.subsystems.climber.config;

import frc.robot.shared.config.AbstractConfig;

/**
 * Configuration bundle for the two front-facing Time of Flight sensors used to align the robot with the tower during endgame.
 * <p>
 * A Time of Flight (ToF) sensor measures distance by timing how long an infrared light pulse takes to bounce off a surface and return. Two sensors
 * mounted side-by-side let the robot detect whether it is centered on the tower and square to it. All thresholds and CAN IDs are exposed through
 * tunable suppliers so they can be adjusted live without redeploying.
 * </p>
 */
public class ClimberSensorConfig extends AbstractConfig {

    /** CAN bus ID of the left Time of Flight sensor. */
    public int    leftSensorCanId                     = 0;

    /** CAN bus ID of the right Time of Flight sensor. */
    public int    rightSensorCanId                    = 0;

    /**
     * Maximum allowable difference between the left and right sensor readings for the robot to be considered aligned to the tower in millimeters.
     */
    public double alignmentToleranceMillimeters       = 25.0;

    /**
     * Readings beyond this distance are treated as "no tower detected" in millimeters.
     */
    public double maximumDetectionDistanceMillimeters = 300.0;

    /**
     * Ranging sample period for the Time of Flight sensors in milliseconds. Lower values provide faster updates but may increase noise.
     */
    public double sampleTimeMilliseconds              = 24.0;

    /**
     * Returns the CAN bus ID of the left Time of Flight sensor.
     *
     * @return left sensor CAN ID
     */
    public int getLeftSensorCanId() {
        return (int) readTunableNumber("leftSensorCanId", leftSensorCanId);
    }

    /**
     * Returns the CAN bus ID of the right Time of Flight sensor.
     *
     * @return right sensor CAN ID
     */
    public int getRightSensorCanId() {
        return (int) readTunableNumber("rightSensorCanId", rightSensorCanId);
    }

    /**
     * Returns the maximum allowable difference between left and right sensor readings for alignment in millimeters.
     *
     * @return alignment tolerance in millimeters
     */
    public double getAlignmentToleranceMillimeters() {
        return readTunableNumber("alignmentToleranceMillimeters", alignmentToleranceMillimeters);
    }

    /**
     * Returns the maximum detection distance in millimeters. Readings beyond this value indicate no tower is present.
     *
     * @return maximum detection distance in millimeters
     */
    public double getMaximumDetectionDistanceMillimeters() {
        return readTunableNumber("maximumDetectionDistanceMillimeters", maximumDetectionDistanceMillimeters);
    }

    /**
     * Returns the ranging sample period for the Time of Flight sensors in milliseconds.
     *
     * @return sample time in milliseconds
     */
    public double getSampleTimeMilliseconds() {
        return readTunableNumber("sampleTimeMilliseconds", sampleTimeMilliseconds);
    }
}
