package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.SupportingClassess.GeneralUtils;

public class Magazine extends SubsystemBase implements GeneralUtils {

// Any variables/fields used in the constructor must appear before the "INSTANCE" variable
// so that they are initialized before the constructor is called.

    //Create necessary objects
    private WPI_TalonSRX m_magazineCenter;
    private WPI_TalonSRX m_magazineRight;
    private WPI_TalonSRX m_magazineLeft;

    private DigitalInput m_beam;

    //Create and define all standard data types needed
    public Magazine() {
        m_magazineCenter = new WPI_TalonSRX(RobotMap.CAN_MAGAZINE_CENTER_MOTOR);
        m_magazineLeft = new WPI_TalonSRX(RobotMap.CAN_MAGAZINE_LEFT_MOTOR);
        m_magazineRight = new WPI_TalonSRX(RobotMap.CAN_MAGAZINE_RIGHT_MOTOR);


        m_magazineCenter.setInverted(false);
        m_magazineLeft.setInverted(true);
        m_magazineRight.setInverted(false);
    }


    public void setCenterSpeed(double speed) {
        m_magazineCenter.set(speed);
    }

    public void setSideSpeeds(double speed) {
        m_magazineLeft.set(speed);
        m_magazineRight.set(speed);
    }

    @Override
    public void outputToShuffleboard() {
        SmartDashboard.putBoolean("Beam", m_beam.get());
    }

    @Override
    public void teleopInit() {}

    @Override
    public void disable() {
        m_magazineCenter.set(0);
    }
}

