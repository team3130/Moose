package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class MotorIntake extends SubsystemBase {

// Any variables/fields used in the constructor must appear before the "INSTANCE" variable
// so that they are initialized before the constructor is called.

    //Create necessary objects
    private WPI_TalonSRX m_motor;
    private WPI_TalonSRX m_magazine;

    //Create and define all standard data types needed

    public MotorIntake() {
        m_motor = new WPI_TalonSRX(RobotMap.CAN_INTAKE_MOTOR);
        m_magazine = new WPI_TalonSRX(RobotMap.CAN_MAGAZINE_MOTOR);
    }

    public void Magazine_spinny(double speed) {
        m_magazine.set(speed);
    }

    public void spinny(double speed) {
        m_motor.set(speed);
    }

}