package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Intake_Pnuematic extends SubsystemBase {
    private Solenoid m_solenoid;

    public Intake_Pnuematic() {
        m_solenoid = new Solenoid(RobotMap.CAN_PNMMODULE, PneumaticsModuleType.CTREPCM, RobotMap.PNM_INTAKE_ACTUATOR_LEFT);
    }

    public boolean toggleIntake() {
        m_solenoid.toggle();
        return m_solenoid.get();
    }

    public boolean toggled() {
        return m_solenoid.get();
    }
}
