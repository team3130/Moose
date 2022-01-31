package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class WHEEL_GO_BRRRRRRRRRRRRRR extends SubsystemBase {

private WPI_TalonSRX m_BRRR;

    public WHEEL_GO_BRRRRRRRRRRRRRR() {
        m_BRRR = new WPI_TalonSRX(RobotMap.WHEEL_GO_BRRRRRRRRRRRRRRRR);
        m_BRRR.setNeutralMode(NeutralMode.Brake);

    }
    public void setSpeed(double speed) {
        m_BRRR.set(speed);
    }
}

