package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Climber extends SubsystemBase {

    private WPI_TalonSRX m_motor1;

    //Create and define all standard data types needed
    public Climber() {
        m_motor1 = new WPI_TalonSRX(RobotMap.kMotor_Subsystem2candevice);
    }

    public void setSpeed1(double speed) {
        m_motor1.set(speed);
    }

}

