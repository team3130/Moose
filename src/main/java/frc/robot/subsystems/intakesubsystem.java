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
    private WPI_TalonSRX m_motor;
    private Solenoid m_solenoid;

    private WPI_TalonSRX m_Magazine;
    //Create and define all standard data types needed

    public intakesubsystem() {
        m_motor = new WPI_TalonSRX(RobotMap.CAN_INTAKE_MOTOR);
        m_solenoid = new Solenoid(RobotMap.CAN_PNMMODULE, PneumaticsModuleType.CTREPCM, RobotMap.PNM_INTAKE_ACTUATOR_LEFT);
        m_Magazine = new WPI_TalonSRX(RobotMap.KMAGAZINEMOTORCANID);
    }

    public boolean toggleIntake() {
        m_solenoid.toggle();
        return m_solenoid.get();
    }

    public boolean toggled() {
        return m_solenoid.get();
    }

    public void spinny(double speed) {
        m_motor.set(speed);
    }

    

}

