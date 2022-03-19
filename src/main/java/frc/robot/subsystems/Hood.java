package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Hood extends SubsystemBase {

// Any variables/fields used in the constructor must appear before the "INSTANCE" variable
// so that they are initialized before the constructor is called.

    //Create necessary objects
    private WPI_TalonSRX m_motor;
    //Create and define all standard data types needed
    public Hood() {
        m_motor = new WPI_TalonSRX(RobotMap.CAN_HOOD);
        m_motor.setInverted(false);
    }

    public void setSpeed(double speed) {
        m_motor.set(speed);
    }

}

