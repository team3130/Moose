package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.utils.Utils;

public class Hood extends SubsystemBase implements GeneralUtils {

// Any variables/fields used in the constructor must appear before the "INSTANCE" variable
// so that they are initialized before the constructor is called.

    ShuffleboardTab tab = Shuffleboard.getTab("hood");

    //Create necessary objects
    private WPI_TalonSRX m_hood;
    //Create and define all standard data types needed
    public Hood() {
        m_hood = new WPI_TalonSRX(RobotMap.CAN_HOOD_MOTOR);
        m_hood.setInverted(false);

        Utils.configMotionMagic(m_hood, 1024, 1024);
        Utils.configPIDF(m_hood,
                RobotMap.kHoodP,
                RobotMap.kHoodI,
                RobotMap.kHoodD,
                RobotMap.kHoodF);

        tab.add("Angle", 0);
    }

    public void setSpeed(double speed) {
        m_hood.set(speed);
    }

    public void zero() {
        m_hood.set(ControlMode.MotionMagic, 0);
    }

    @Override
    public void outputToShuffleboard() {
        SmartDashboard.putNumber("Angle", m_hood.getSelectedSensorPosition());
    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void disable() {

    }
}

