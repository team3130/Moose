package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Magazine extends SubsystemBase implements SubsystemBased {

// Any variables/fields used in the constructor must appear before the "INSTANCE" variable
// so that they are initialized before the constructor is called.

    //Create necessary objects
    private WPI_TalonSRX m_magazine;

    //Create and define all standard data types needed
    public Magazine() {
        m_magazine = new WPI_TalonSRX(RobotMap.CAN_MAGAZINE_MOTOR);
        m_magazine.setInverted(true);
    }

    public void setSpeed(double speed) {
        m_magazine.set(speed);
    }

    @Override
    public void outputToShuffleboard() {}

    @Override
    public void teleopInit() {}

    @Override
    public void disable() {
        m_magazine.set(0);
    }
}

