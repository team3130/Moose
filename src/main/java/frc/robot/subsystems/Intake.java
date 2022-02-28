package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase {

// Any variables/fields used in the constructor must appear before the "INSTANCE" variable
// so that they are initialized before the constructor is called.

    //Create necessary objects
    private WPI_TalonSRX m_motor;
    private Solenoid m_solenoid;

    //Create and define all standard data types needed

    public Intake() {
        m_motor = new WPI_TalonSRX(RobotMap.CAN_INTAKE_MOTOR);
        m_solenoid = new Solenoid(RobotMap.CAN_PNMMODULE, PneumaticsModuleType.CTREPCM, RobotMap.PNM_INTAKE_ACTUATOR_LEFT);

        m_motor.setInverted(true);
    }

    public void setSpeed(double speed) {
        m_motor.set(speed);
    }

    public boolean toggleIntake() {
        m_solenoid.toggle();
        return m_solenoid.get();
    }

    public boolean toggled() {
        return m_solenoid.get();
    }

    public void deployIntake(boolean deploy) {
        m_solenoid.set(deploy);
    }
}