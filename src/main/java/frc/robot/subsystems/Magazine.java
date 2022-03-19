package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Magazine extends SubsystemBase implements GeneralUtils {

// Any variables/fields used in the constructor must appear before the "INSTANCE" variable
// so that they are initialized before the constructor is called.

    //Create necessary objects
    private WPI_TalonSRX m_magazine;

    private DigitalInput m_beam;

    //Create and define all standard data types needed
    public Magazine() {
        m_magazine = new WPI_TalonSRX(RobotMap.CAN_MAGAZINE_MOTOR);
        m_magazine.setInverted(true);

        m_beam = new DigitalInput(RobotMap.DIO_FEEDERBEAM);
    }

    public boolean isEmpty() {
        // if the beam returns true, the beam is intact and there is therefore no ball breaking the beam
        //TODO: Sanity check
        return m_beam.get();
    }

    public void setSpeed(double speed) {
        m_magazine.set(speed);
    }

    @Override
    public void outputToShuffleboard() {
        SmartDashboard.putBoolean("Beam", m_beam.get());
    }

    @Override
    public void teleopInit() {}

    @Override
    public void disable() {
        m_magazine.set(0);
    }
}

