package frc.robot.subsystems.climber.io;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import frc.robot.subsystems.climber.config.ClimberSensorConfig;

/**
 * Real hardware implementation that reads distance values from two PlayingWithFusion CAN Time of Flight sensors.
 * <p>
 * Each sensor is configured with short ranging mode and the sample period from {@link ClimberSensorConfig}. The sensors are mounted as a front-facing
 * pair so the robot can detect and center itself on the tower for endgame climbing.
 * </p>
 */
public class ClimberSensorIOReal implements ClimberSensorIO {

    /** Left Time of Flight sensor connected via CAN bus. */
    private final TimeOfFlight leftSensor;

    /** Right Time of Flight sensor connected via CAN bus. */
    private final TimeOfFlight rightSensor;

    /**
     * Creates the real sensor IO and configures both Time of Flight sensors with the specified ranging mode and sample period.
     *
     * @param config sensor configuration containing CAN IDs and sample timing
     */
    public ClimberSensorIOReal(ClimberSensorConfig config) {
        leftSensor  = new TimeOfFlight(config.getLeftSensorCanId());
        rightSensor = new TimeOfFlight(config.getRightSensorCanId());

        int sampleTime = (int) config.getSampleTimeMilliseconds();
        leftSensor.setRangingMode(RangingMode.Short, sampleTime);
        rightSensor.setRangingMode(RangingMode.Short, sampleTime);
    }

    @Override
    public void updateInputs(ClimberSensorIOInputs inputs) {
        double leftRange  = leftSensor.getRange();
        double rightRange = rightSensor.getRange();

        inputs.leftDistanceMillimeters  = leftRange;
        inputs.rightDistanceMillimeters = rightRange;

        // A valid reading is a positive, non-zero distance.
        inputs.leftSensorConnected      = leftRange > 0.0;
        inputs.rightSensorConnected     = rightRange > 0.0;
    }
}
