package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood_Pnuematics extends SubsystemBase {

// Any variables/fields used in the constructor must appear before the "INSTANCE" variable
// so that they are initialized before the constructor is called.

    //Create necessary objects

    private Solenoid m_solenoid;

    //Create and define all standard data types needed
    public Hood_Pnuematics() {
        m_solenoid = new Solenoid(RobotMap.CAN_PNMMODULE, PneumaticsModuleType.CTREPCM, RobotMap.PNM_HOOD_ACTUATOR);
    }

    public boolean toggleIntake() {
        m_solenoid.toggle();
        return m_solenoid.get();
    }

    public boolean toggled() {
        return m_solenoid.get();

}

