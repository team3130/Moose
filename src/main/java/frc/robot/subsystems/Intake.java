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
    private static Solenoid m_intakeSolenoid;
    private static Solenoid m_intakeSolenoid2;
    private static boolean deploy = false;

    private static final Intake instance = new Intake();

    //Create and define all standard data types needed
    public static Intake getInstance() {
        return instance;
    }

    public Intake() {
        m_motor = new WPI_TalonSRX(RobotMap.CAN_SHOOTER_MOTOR);
        m_intakeSolenoid = new Solenoid(RobotMap.CAN_PNMMODULE, PneumaticsModuleType.CTREPCM, RobotMap.PNM_INTAKE);
        m_intakeSolenoid2 = new Solenoid(RobotMap.CAN_PNMMODULE, PneumaticsModuleType.CTREPCM, RobotMap.PNM_INTAKE);
    }

    public static void deployIntake() {
        m_intakeSolenoid.set(true);
        m_intakeSolenoid2.set(true);
    }

    public static void retractIntake() {
        m_intakeSolenoid.set(false);
        m_intakeSolenoid2.set(false);
    }

    public static void toggleIntake() {
        m_intakeSolenoid.set(!m_intakeSolenoid.get());
        m_intakeSolenoid2.set(!m_intakeSolenoid2.get());
    }

    public void spinMotor(double speed) {
        if (!m_intakeSolenoid.get() && !m_intakeSolenoid2.get()) {
            m_motor.set(speed);
        } else {
            m_motor.set(0);
        }
    }
}

