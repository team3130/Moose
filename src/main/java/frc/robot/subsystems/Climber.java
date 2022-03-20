package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Climber extends SubsystemBase {

// Any variables/fields used in the constructor must appear before the "INSTANCE" variable
// so that they are initialized before the constructor is called.

    //Create necessary objects
    private WPI_TalonSRX m_climber_motor;
    private WPI_TalonSRX m_climber_motor_follower;
    private Solenoid m_solenoid;

    //Create and define all standard data types needed
    public Climber() {
        m_climber_motor = new WPI_TalonSRX(RobotMap.CAN_CLIMBER_LEFT);
        m_climber_motor_follower = new WPI_TalonSRX(RobotMap.CAN_CLIMBER_RIGHT);
        m_climber_motor_follower.follow(m_climber_motor);

        m_solenoid = new Solenoid(RobotMap.CAN_PNMMODULE, PneumaticsModuleType.CTREPCM, RobotMap.PNM_CLIMBER_ACTUATOR);
    }

    public void setSpeed(double speed) {
        m_climber_motor.set(speed);
    }

    public boolean isDeployed() {
        return m_solenoid.get();
    }

    public void deployClimber() {
        m_solenoid.toggle();
    }
}

