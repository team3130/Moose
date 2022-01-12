package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {
    private WPI_TalonSRX m_motor;

    private ShuffleboardTab tab = Shuffleboard.getTab("Motor");

    private NetworkTableEntry sped = tab.add("Speed", 0.5).getEntry();

    //Create and define all standard data types needed
    public Shooter() {
        m_motor = new WPI_TalonSRX(RobotMap.CAN_SHOOTER_MOTOR);
    }

    public void spinMotor(double speed) {
        m_motor.set(speed);
    }

    public double getSpeedFromShuffleboard() {
        return sped.getDouble(0.5);
    }

}

