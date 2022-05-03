package frc.robot.controls;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;

public class PivotTrigger extends Button {

    private Joystick joystick;
    private int axis;

    public PivotTrigger(Joystick joystick, int buttonNumber) {
        this.joystick = joystick;
        this.axis = buttonNumber;
    }

    @Override
    public boolean get() {
        return (this.axis == joystick.getPOV());
    }
}
