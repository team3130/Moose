package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {

// Any variables/fields used in the constructor must appear before the "INSTANCE" variable
// so that they are initialized before the constructor is called.

    //Create necessary objects
    private  WPI_TalonSRX m_motor;
    //Create and define all standard data types needed
    public grapplingHook() {
        m_motor = new WPI_TalonSRX(RobotMap.kgrapplingHook);
        m_motor.setNeutralMode(NeutralMode.Brake);
    }
    public void setSpeed(double speed) {
        m_motor.set(speed);
    }
}
