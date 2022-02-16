package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;


public class Climber extends SubsystemBase {

// Any variables/fields used in the constructor must appear before the "INSTANCE" variable
// so that they are initialized before the constructor is called.

    //Create necessary objects
    private WPI_TalonFX m_motor;
    //Create and define all standard data types needed
    public Climber() {
        m_motor = new WPI_TalonFX(RobotMap.CAN_CLIMBER);
    }

    public void spinMotor(double speed) {
        m_motor.set(speed);
    }

}

