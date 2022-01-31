package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class FlyWheel extends SubsystemBase{

private WPI_TalonSRX m_flywheel;

    public FlyWheel() {
        m_flywheel = new WPI_TalonSRX(RobotMap.FlyWheel);
        m_flywheel.setNeutralMode(NeutralMode.Coast);
    }
    public void setacceleration(double acceleration) {m_flywheel.set(acceleration);}
}
