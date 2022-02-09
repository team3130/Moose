package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Climber extends SubsystemBase {

// Any variables/fields used in the constructor must appear before the "INSTANCE" variable
// so that they are initialized before the constructor is called.

    //Create necessary objects
    private Solenoid m_pnuematic;

    //Create and define all standard data types needed
    public Climber() {
        m_pnuematic = new Solenoid(RobotMap.CAN_PNU_MATIC_HUB, PneumaticsModuleType.CTREPCM, RobotMap.CLIMBER_ACTUATOR);
    }

    public boolean toggleClimber() {
        m_pnuematic.toggle();
        return m_pnuematic.get();
    }

}

