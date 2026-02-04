package frc.robot.devices;

import java.util.function.Function;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Convenience wrapper for a generic game controller. Provides named buttons and axes plus helper methods to bind commands.
 */
public class GameController extends CommandJoystick {

    /**
     * Buttons on a standard game controller.
     */
    public enum GameControllerButton {
        /** Button A. */
        A(1),
        /** Button B. */
        B(2),
        /** Button X. */
        X(3),
        /** Button Y. */
        Y(4),
        /** Left bumper. */
        L1(5),
        /** Right bumper. */
        R1(6),
        /** Back/select button. */
        Back(7),
        /** Start button. */
        Start(8),
        /** Left stick press. */
        LStick(9),
        /** Right stick press. */
        RStick(10);

        private final int value;

        GameControllerButton(int value) {
            this.value = value;
        }

        /**
         * Returns the raw driver station button index.
         *
         * @return button index used by WPILib
         */
        public int getValue() {
            return value;
        }
    }

    /**
     * Axes on a standard game controller.
     */
    public enum GameControllerAxes {
        /** Left stick X axis. */
        LeftStickX(0),
        /** Left stick Y axis. */
        LeftStickY(1),
        /** Right stick X axis. */
        RightStickX(4),
        /** Right stick Y axis. */
        RightStickY(5),
        /** Left trigger axis. */
        LTrigger(2),
        /** Right trigger axis. */
        RTrigger(3);

        private final int value;

        GameControllerAxes(int value) {
            this.value = value;
        }

        /**
         * Returns the raw driver station axis index.
         *
         * @return axis index used by WPILib
         */
        public int getValue() {
            return value;
        }
    }

    /**
     * Creates a game controller wrapper for the given USB port.
     *
     * @param port driver station USB port index
     */
    public GameController(int port) {
        super(port);
    }

    /**
     * Runs a command while the given button stays held.
     *
     * @param button           controller button to watch
     * @param whileHeldCommand command to run the entire time the button is held
     * @return trigger representing the hold state
     */
    public Trigger onButtonHold(GameControllerButton button, Command whileHeldCommand) {
        return new Trigger(button(button.getValue())).whileTrue(whileHeldCommand);
    }

    /**
     * Runs a command while the given button ID stays held.
     *
     * @param buttonId         numeric button ID (see driver station mapping)
     * @param whileHeldCommand command to run the entire time the button is held
     * @return trigger representing the hold state
     */
    public Trigger onButtonHold(int buttonId, Command whileHeldCommand) {
        return new Trigger(button(buttonId)).whileTrue(whileHeldCommand);
    }

    /**
     * Runs a command while the given button stays held, then runs another when released.
     *
     * @param button              controller button to watch
     * @param whileHeldCommand    command to run the entire time the button is held
     * @param whenReleasedCommand command to run once the button is released
     * @return trigger representing the hold state
     */
    public Trigger onButtonHold(GameControllerButton button, Command whileHeldCommand, Command whenReleasedCommand) {
        return new Trigger(button(button.getValue())).whileTrue(whileHeldCommand).onFalse(whenReleasedCommand);
    }

    /**
     * Runs a command while the given button ID stays held, then runs another when released.
     *
     * @param buttonId            numeric button ID (see driver station mapping)
     * @param whileHeldCommand    command to run the entire time the button is held
     * @param whenReleasedCommand command to run once the button is released
     * @return trigger representing the hold state
     */
    public Trigger onButtonHold(int buttonId, Command whileHeldCommand, Command whenReleasedCommand) {
        return new Trigger(button(buttonId)).whileTrue(whileHeldCommand).onFalse(whenReleasedCommand);
    }

    /**
     * Runs a command once when the given button is pressed.
     *
     * @param button         controller button to watch
     * @param onPressCommand command to schedule on press
     * @return trigger representing the press event
     */
    public Trigger onButtonPress(GameControllerButton button, Command onPressCommand) {
        return new Trigger(button(button.getValue())).onTrue(onPressCommand);
    }

    /**
     * Runs a command once when the given button ID is pressed.
     *
     * @param buttonId       numeric button ID (see driver station mapping)
     * @param onPressCommand command to schedule on press
     * @return trigger representing the press event
     */
    public Trigger onButtonPress(int buttonId, Command onPressCommand) {
        return new Trigger(button(buttonId)).onTrue(onPressCommand);
    }

    /**
     * Runs commands when the given button is pressed and when it is released.
     *
     * @param button           controller button to watch
     * @param onPressCommand   command to schedule on press
     * @param onReleaseCommand command to schedule on release
     * @return trigger representing the press event
     */
    public Trigger onButtonPress(GameControllerButton button, Command onPressCommand, Command onReleaseCommand) {
        return onButtonPress(button, onPressCommand).onFalse(onReleaseCommand);
    }

    /**
     * Runs commands when the given button ID is pressed and when it is released.
     *
     * @param buttonId         numeric button ID (see driver station mapping)
     * @param onPressCommand   command to schedule on press
     * @param onReleaseCommand command to schedule on release
     * @return trigger representing the press event
     */
    public Trigger onButtonPress(int buttonId, Command onPressCommand, Command onReleaseCommand) {
        return onButtonPress(buttonId, onPressCommand).onFalse(onReleaseCommand);
    }

    /**
     * Runs a command while an axis satisfies the given condition. The axis value is polled each time the trigger updates.
     *
     * @param axis              controller axis to monitor
     * @param conditionFunction predicate that returns true when the axis should trigger
     * @param whileTrueCommand  command to run while the predicate returns true
     * @return trigger representing the axis condition
     */
    public Trigger onAxisChange(GameControllerAxes axis, Function<Double, Boolean> conditionFunction, Command whileTrueCommand) {
        return new Trigger(() -> conditionFunction.apply(getRawAxis(axis.getValue()))).whileTrue(whileTrueCommand);
    }

    /**
     * Runs commands while an axis satisfies the given condition and when it does not. The axis value is polled each time the trigger updates.
     *
     * @param axis              controller axis to monitor
     * @param conditionFunction predicate that returns true when the axis should trigger
     * @param whileTrueCommand  command to run while the predicate returns true
     * @param whileFalseCommand command to run while the predicate returns false
     * @return trigger representing the axis condition
     */
    public Trigger onAxisChange(
            GameControllerAxes axis,
            Function<Double, Boolean> conditionFunction,
            Command whileTrueCommand,
            Command whileFalseCommand) {
        return onAxisChange(axis, conditionFunction, whileTrueCommand).onFalse(whileFalseCommand);
    }
}
