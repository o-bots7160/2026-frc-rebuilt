package frc.robot.subsystems.harvester.config;

import frc.robot.shared.config.AbstractSetAndSeekSubsystemConfig;

/**
 * Configuration bundle for the harvester arm mechanism. Values are stored in degrees for readability but converted to radians at runtime where
 * needed.
 * <p>
 * The harvester arm swings the intake rollers between an upright stowed position (inside the robot perimeter) and a lowered deployed position
 * (outside the perimeter to grab Fuel from the floor). Named position fields provide the default stowed and deployed angles so commands do not
 * hard-code magic numbers.
 * </p>
 * <p>
 * Because the arm fights gravity, this config adds a {@code kG} gain for use with {@link edu.wpi.first.math.controller.ArmFeedforward}. The
 * {@code horizontalOffsetDegrees} field maps the encoder's zero position to the angle from horizontal so the gravity term is computed correctly.
 * </p>
 */
public class HarvesterSubsystemConfig extends AbstractSetAndSeekSubsystemConfig {

    /** Arm angle when stowed upright inside the robot perimeter, in degrees. This is the match-start and default position. */
    public double               stowedPositionDegrees;

    /** Arm angle when deployed downward outside the robot perimeter to collect Fuel, in degrees. */
    public double               deployedPositionDegrees;

    /** Arm angle raised to push Fuel from the back of the hopper toward the shooting array, in degrees. */
    public double               sweepPositionDegrees;

    /**
     * Angle offset from the encoder's zero position to the arm's horizontal reference, in degrees.
     * <p>
     * WPILib's {@link edu.wpi.first.math.controller.ArmFeedforward} expects the arm angle measured from horizontal (0 degrees = parallel to the
     * floor). If the encoder already reads zero when the arm is horizontal (deployed), the offset is 0 degrees. If the encoder reads zero when the
     * arm is vertical (stowed upright), the offset would be 90 degrees.
     * </p>
     */
    public double               horizontalOffsetDegrees;

    /**
     * Maximum allowed drift from the deployed position before the hold command re-engages, in degrees. A ball pushing the arm upward beyond this
     * threshold triggers a profiled return to the deployed angle.
     */
    public double               deployedHoldToleranceDegrees;

    /**
     * Returns the stowed arm position, tuned via SmartDashboard.
     *
     * @return stowed position in degrees (arm upright inside the frame)
     */
    public double getStowedPositionDegrees() {
        return readTunableDegrees("stowedPositionDegrees", stowedPositionDegrees);
    }

    /**
     * Returns the deployed arm position, tuned via SmartDashboard.
     *
     * @return deployed position in degrees (arm lowered outside the frame to collect Fuel)
     */
    public double getDeployedPositionDegrees() {
        return readTunableDegrees("deployedPositionDegrees", deployedPositionDegrees);
    }

    /**
     * Returns the sweep arm position, tuned via SmartDashboard.
     *
     * @return sweep position in degrees (arm raised to push Fuel toward the shooting array)
     */
    public double getSweepPositionDegrees() {
        return readTunableDegrees("sweepPositionDegrees", sweepPositionDegrees);
    }

    /**
     * Returns the horizontal offset angle in radians.
     *
     * @return offset in radians added to the encoder position to obtain the angle from horizontal
     */
    public double getHorizontalOffsetRadians() {
        return readTunableDegreesAsRadians("horizontalOffsetDegrees", horizontalOffsetDegrees);
    }

    /**
     * Returns the maximum drift from deployed position before re-engaging the motor, tuned via SmartDashboard.
     *
     * @return hold tolerance in degrees
     */
    public double getDeployedHoldToleranceDegrees() {
        return readTunableDegrees("deployedHoldToleranceDegrees", deployedHoldToleranceDegrees);
    }
}
