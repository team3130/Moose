package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.utils.Utils;

public class Hood extends SubsystemBase {
    private WPI_TalonSRX m_hood;

    private ShuffleboardTab tab = Shuffleboard.getTab("Hood");
    private NetworkTableEntry readHoodAngle = tab.add("Read Hood Angle", 0.0).getEntry();
    private NetworkTableEntry writeHoodAngle = tab.add("Write Hood Angle", 0.0).getEntry();
// Any variables/fields used in the constructor must appear before the "INSTANCE" variable
// so that they are initialized before the constructor is called.

    //Create necessary objects

    //Create and define all standard data types needed
    public Hood() {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        m_hood = new WPI_TalonSRX(RobotMap.CAN_HOOD);
        m_hood.configFactoryDefault();

        m_hood.setInverted(false); //TODO: find real
        m_hood.setSensorPhase(true);

        m_hood.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

        m_hood.configForwardSoftLimitThreshold(45.0 * RobotMap.kHoodTicksPerDegree);
        m_hood.configReverseSoftLimitThreshold(0);

        m_hood.configForwardSoftLimitEnable(true);
        m_hood.configReverseSoftLimitEnable(true);

        m_hood.clearStickyFaults();

        m_hood.set(ControlMode.PercentOutput, 0.0);

        Utils.configPIDF(m_hood,
                RobotMap.kHoodP,
                RobotMap.kHoodI,
                RobotMap.kHoodD,
                RobotMap.kHoodF);

        Utils.configMotionMagic(m_hood, 1, 1); //TODO: determine whether velocity/accel param are arbitrary
    }

    public synchronized void setAngle(double angleDeg){
        m_hood.set(ControlMode.MotionMagic, angleDeg * RobotMap.kHoodTicksPerDegree);
    }

    /**
     *
     * @return the current target angle in degrees
     */
    public double getAngleSetpoint(){
        return m_hood.getClosedLoopTarget() / RobotMap.kHoodTicksPerDegree;
    }

    public double getRelativeHoodAngle(){
        return m_hood.getSelectedSensorPosition() / RobotMap.kHoodTicksPerDegree;
    }

    public boolean canShoot(){
        return(Math.abs(getAngleSetpoint() - getRelativeHoodAngle()) <= 2);
        //2 degrees of lenience
    }

    public double getShuffleboardInput(){
        return writeHoodAngle.getDouble(0);
    }

    @Override
    public void outputToShuffleBoard(){
        readHoodAngle.setNumber(getRelativeHoodAngle());

    }
}


