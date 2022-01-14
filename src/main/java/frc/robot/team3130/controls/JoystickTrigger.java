package frc.robot.team3130.controls;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class JoystickTrigger extends Trigger {

    private GenericHID stick;
    private int axis;
    private double threshold;

    /**
     * Creates a joystick button like object from a gamepad trigger axis.
     *
     * @param joystick     The GenericHID object that has the button
     *                     (e.g. Joystick, KinectStick, etc)
     * @param axisNumber The axis number of the trigger
     */
    public JoystickTrigger(GenericHID joystick, int axisNumber) {
        this.stick = joystick;
        this.axis = axisNumber;
        threshold = 0.1;
    }

    /**
     * Creates a joystick button like object from a gamepad trigger axis.
     *
     * @param joystick     The GenericHID object that has the button
     *                     (e.g. Joystick, KinectStick, etc)
     * @param axisNumber The axis number of the trigger
     * @param threshold  The threshold the trigger should reach to trigger
     */
    public JoystickTrigger(GenericHID joystick, int axisNumber, double threshold) {
        this.stick = joystick;
        this.axis = axisNumber;
        this.threshold = threshold;
    }

    @Override
    public boolean get() {
        return stick.getRawAxis(axis) > threshold;
    }

    /**
     * Starts the given command when the button is first pressed, and cancels it when it is released,
     * but does not start it again if it ends or is otherwise interrupted.
     *
     * @param command       the command to start
     * @return this button, so calls can be chained
    */
    public JoystickTrigger whenHeld(final Command command) {
        whileActiveOnce(command, true);
        return this;
    }
}