package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class motorsub extends SubsystemBase {

// Any variables/fields used in the constructor must appear before the "INSTANCE" variable
// so that they are initialized before the constructor is called.

    //Create necessary objects
    private WPI_TalonSRX m_motor1;

    //Create and define all standard data types needed
    public motorsub() {
        m_motor1 = new WPI_TalonSRX(RobotMap.kmotorsubcandevice);

}

    public void setspeed(double speed) {
            m_motor1.set(speed);
    }

}

