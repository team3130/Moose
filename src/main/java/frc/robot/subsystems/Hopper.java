package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {

// Any variables/fields used in the constructor must appear before the "INSTANCE" variable
// so that they are initialized before the constructor is called.

    //Create necessary objects

    private WPI_TalonFX m_motor2;

    private static final Hopper instance = new Hopper();

    //Create and define all standard data types needed
    public static Hopper getInstance() {
        return instance;
    }
    
    public Hopper() {
        
    }

    public void setSpeed(double speed) {
        m_motor2.set(speed);
    }

}

