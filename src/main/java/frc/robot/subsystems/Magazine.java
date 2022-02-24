package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Magazine extends SubsystemBase {

// Any variables/fields used in the constructor must appear before the "INSTANCE" variable
// so that they are initialized before the constructor is called.

    //Create necessary objects
    private WPI_TalonSRX m_Magazine;

    //Create and define all standard data types needed
    public Magazine() {
        m_Magazine = new WPI_TalonSRX(RobotMap.KMAGAZINEMOTORCANID);
    }

    public void spinny(double speed) {
        m_Magazine.set(speed);
    }
}

