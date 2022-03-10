package frc.robot.subsystems;

import com.ctre.phoenix.Util;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Magazine extends SubsystemBase implements GeneralUtils {

// Any variables/fields used in the constructor must appear before the "INSTANCE" variable
// so that they are initialized before the constructor is called.

    //Create necessary objects
    private WPI_TalonSRX m_center;
    private WPI_TalonSRX m_hopperL;
    private WPI_TalonSRX m_hopperR;

    //Create and define all standard data types needed
    public Magazine() {
        m_center = new WPI_TalonSRX(RobotMap.CAN_MAGAZINE_MOTOR);
        m_hopperL = new WPI_TalonSRX(RobotMap.CAN_HOPPER_L_MOTOR);
        m_hopperR = new WPI_TalonSRX(RobotMap.CAN_HOPPER_R_MOTOR);
        m_center.setInverted(true);
        m_hopperR.setInverted(true);
        m_hopperL.setInverted(false);
    }

    public void setCenterSpeed(double speed) {
        m_center.set(speed);
    }

    public void setHopperSpeed(double speed) {
        m_hopperL.set(-1 * speed);
        m_hopperR.set(speed);
    }

    @Override
    public void outputToShuffleboard() {}

    @Override
    public void teleopInit() {}

    @Override
    public void disable() {
        m_center.set(0);
        m_hopperL.set(0);
        m_hopperR.set(0);
    }
}

