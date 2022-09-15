package frc.robot.controls;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

class POVTrigger extends Trigger {

    private Joystick stick;
    private int POV;

    public POVTrigger(Joystick stick, int POV) {
        this.stick = stick;
        this.POV = POV;
    }

    @Override
    public boolean get() {
        return stick.getPOV(0) == POV;
    }

}