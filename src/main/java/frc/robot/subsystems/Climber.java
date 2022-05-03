package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.SupportingClassess.GeneralUtils;
import frc.robot.utils.Utils;

public class Climber extends SubsystemBase implements GeneralUtils {

// Any variables/fields used in the constructor must appear before the "INSTANCE" variable
// so that they are initialized before the constructor is called.
    //Create necessary objects
    private WPI_TalonFX m_left_motor;
    private WPI_TalonFX m_right_motor;
    private Solenoid m_solenoid;
    private DigitalInput m_rightlimitswitch;
    private DigitalInput m_leftlimitswitch;

    private int LeftFullExtension = 226893;
    private int RightFullExtension = 220302;

    //Create and define all standard data types needed
    public Climber() {
        m_left_motor = new WPI_TalonFX(RobotMap.CAN_CLIMBER_LEFT);
        m_right_motor = new WPI_TalonFX(RobotMap.CAN_CLIMBER_RIGHT);
        m_rightlimitswitch = new DigitalInput(RobotMap.RIGHT_LIMITSWITCH);
        m_leftlimitswitch = new DigitalInput(RobotMap.LEFT_LIMITSWITCH);

        m_right_motor.setInverted(true);
        m_left_motor.setInverted(true);

        m_right_motor.setNeutralMode(NeutralMode.Brake);
        m_left_motor.setNeutralMode(NeutralMode.Brake);

        m_solenoid = new Solenoid(RobotMap.CAN_PNMMODULE, PneumaticsModuleType.CTREPCM, RobotMap.PNM_CLIMBER_ACTUATOR);

        Utils.configPIDF(m_left_motor, RobotMap.kClimberP, RobotMap.kClimberI, RobotMap.kClimberD, RobotMap.kClimberS);
        Utils.configPIDF(m_right_motor, RobotMap.kClimberP, RobotMap.kClimberI, RobotMap.kClimberD, RobotMap.kClimberS);
    }

    public void setSpeedLeft(double speed) {
        m_left_motor.set(speed);
    }

    public void setSpeedRight(double speed) {
        m_right_motor.set(speed);
    }

    public void zero() {
        m_left_motor.set(ControlMode.PercentOutput, -0.25);
        m_right_motor.set(ControlMode.PercentOutput, -0.25);
    }

    public boolean isDeployed() {
        return m_solenoid.get();
    }

    public void deployClimber() {
        m_solenoid.toggle();
    }

    public void configRampRate(double rampSeconds){
        m_left_motor.configOpenloopRamp(rampSeconds);
        m_right_motor.configOpenloopRamp(rampSeconds);
    }

    public void automateClimber() {
        m_left_motor.set(ControlMode.MotionMagic, LeftFullExtension);
        m_right_motor.set(ControlMode.MotionMagic, RightFullExtension);
    }

    public boolean checkExtendFin() {
        return m_left_motor.isMotionProfileFinished() && m_right_motor.isMotionProfileFinished();
    }

    public boolean brokeLeft() {
        return !m_leftlimitswitch.get();
    }

    public boolean brokeRight() {
        return !m_rightlimitswitch.get();
    }

    public void resetEncodersLeft() {
        m_left_motor.setSelectedSensorPosition(0);
    }

    public void resetEncodersRight() {
        m_right_motor.setSelectedSensorPosition(0);
    }

    public void outputToShuffleboard() {
        SmartDashboard.putBoolean("limit switch right:", brokeRight());
        SmartDashboard.putBoolean("limit switch left", brokeLeft());
        SmartDashboard.putNumber("left encoder reading", m_left_motor.getSelectedSensorPosition());
        SmartDashboard.putNumber("right encoder reading", m_right_motor.getSelectedSensorPosition());
    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void disable() {

    }

    public void stop() {
        setSpeedLeft(0);
        setSpeedRight(0);
    }

    public double getPosL() {
        return m_left_motor.getSelectedSensorPosition();
    }

    public double getPosR() {
        return m_right_motor.getSelectedSensorPosition();
    }
}

