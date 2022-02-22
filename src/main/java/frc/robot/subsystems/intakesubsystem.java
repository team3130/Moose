package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class intakesubsystem extends SubsystemBase {

// Any variables/fields used in the constructor must appear before the "INSTANCE" variable
// so that they are initialized before the constructor is called.

    //Create necessary objects
    private WPI_TalonSRX m_Motor;
    private Solenoid m_RightPnuematic;
    private Solenoid m_LeftPnuematic;

    //Create and define all standard data types needed

    public intakesubsystem() {
        m_Motor = new WPI_TalonSRX(RobotMap.KINTAKEMOTORCANID);
        m_LeftPnuematic = new Solenoid(RobotMap.CAN_PNU_MATIC_HUB, PneumaticsModuleType.CTREPCM, RobotMap.INTAKE_ACTUATOR_LEFT);
        m_RightPnuematic = new Solenoid(RobotMap.CAN_PNU_MATIC_HUB, PneumaticsModuleType.CTREPCM, RobotMap.INTAKE_ACTUATOR_RIGHT);

    }

    public boolean toggleIntake() {
        assert m_RightPnuematic.get() == m_LeftPnuematic.get();
        m_LeftPnuematic.toggle();
        m_RightPnuematic.toggle();
        return m_RightPnuematic.get();
    }
    public boolean toggled() {
        return m_RightPnuematic.get();
    }

    public void spinny(double speed){
        m_Motor.set(speed);
    }

}

