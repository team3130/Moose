package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.utils.Utils;

public class Climber extends SubsystemBase {

// Any variables/fields used in the constructor must appear before the "INSTANCE" variable
// so that they are initialized before the constructor is called.
    //Create necessary objects
    private WPI_TalonFX m_climber_motor;
    private WPI_TalonFX m_climber_motor_follower;
    private Solenoid m_solenoid;

    private DifferentialDrive drive;

    //Create and define all standard data types needed
    public Climber() {
        m_climber_motor = new WPI_TalonFX(RobotMap.CAN_CLIMBER_LEFT);
        m_climber_motor_follower = new WPI_TalonFX(RobotMap.CAN_CLIMBER_RIGHT);

        m_climber_motor_follower.setInverted(true);
        m_climber_motor.setInverted(true);
       // m_climber_motor_follower.follow(m_climber_motor);

        m_solenoid = new Solenoid(RobotMap.CAN_PNMMODULE, PneumaticsModuleType.CTREPCM, RobotMap.PNM_CLIMBER_ACTUATOR);

        drive = new DifferentialDrive(m_climber_motor, m_climber_motor_follower);

        Utils.configPIDF(m_climber_motor, RobotMap.kClimberP, RobotMap.kClimberI, RobotMap.kClimberD, RobotMap.kClimberS);
        Utils.configPIDF(m_climber_motor_follower, RobotMap.kClimberP, RobotMap.kClimberI, RobotMap.kClimberD, RobotMap.kClimberS);
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

    public void driveTank(double left, double right, boolean squared) {
        drive.tankDrive(left, right, squared);
    }

    public void configRampRate(double rampSeconds){
        m_climber_motor.configOpenloopRamp(rampSeconds);
        m_climber_motor_follower.configOpenloopRamp(rampSeconds);
    }

    public void automateClimber() {
        m_climber_motor.set(ControlMode.MotionMagic, 100069);
        m_climber_motor_follower.set(ControlMode.MotionMagic, 100069);
    }
}

