package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class MotorThing2 extends SubsystemBase {

// Any variables/fields used in the constructor must appear before the "INSTANCE" variable
// so that they are initialized before the constructor is called.

    //Create necessary objects
    private WPI_TalonFX m_motor;
    //Create and define all standard data types needed
    public MotorThing2() {
       m_motor = new WPI_TalonFX(RobotMap.kMotorThingCanDevice);
       m_motor.setNeutralMode(NeutralMode.Brake);
    }
public void setSpeed(double speed) {
        m_motor.set(speed);
}
}

