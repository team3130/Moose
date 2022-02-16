package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Climber extends SubsystemBase {

// Any variables/fields used in the constructor must appear before the "INSTANCE" variable
// so that they are initialized before the constructor is called.

    //Create necessary objects
    private WPI_TalonSRX motor1, motor2;

    //Create and define all standard data types needed
    public Climber() {
        motor1 = new WPI_TalonSRX(RobotMap.climber_moter1_int);
        motor2 = new WPI_TalonSRX(RobotMap.climber_moter2_int);

        motor2.follow(motor1);
    }

    public void spin(double sped) {
        motor1.set(sped);
    }
}

