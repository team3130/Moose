package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ChassisCooler extends SubsystemBase {
    //Create necessary objects
    private final Solenoid m_cooler;

    private boolean inverted;

    //Create and define all standard data types needed
    public ChassisCooler() {
        m_cooler = new Solenoid(RobotMap.CAN_PNMMODULE, PneumaticsModuleType.CTREPCM, RobotMap.PNM_CHASSIS_COOLER);
        m_cooler.set(false);
    }

    public void ToggleCooler () {
        m_cooler.toggle();
    }

    public void SetCooler(boolean Cool) {
        m_cooler.set(inverted^Cool);
    }

    public Boolean GetCooler () {
        return inverted^m_cooler.get();
    }

}

