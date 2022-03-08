package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.utils.Utils;

public class Hood extends SubsystemBase implements GeneralUtils {
    private WPI_TalonSRX m_hood;

    private ShuffleboardTab tab = Shuffleboard.getTab("Hood");
    private NetworkTableEntry readHoodAngle = tab.add("Read Hood Angle", 0.0).getEntry();
    private NetworkTableEntry writeHoodAngle = tab.add("Write Hood Angle", 0.0).getEntry();
// Any variables/fields used in the constructor must appear before the "INSTANCE" variable
// so that they are initialized before the constructor is called.

    //Create necessary objects

    //Create and define all standard data types needed
    public Hood() {


        m_hood = new WPI_TalonSRX(RobotMap.CAN_HOOD);
        m_hood.configFactoryDefault();

        m_hood.setInverted(false); //TODO: find real
        m_hood.setSensorPhase(true);

        m_hood.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        m_hood.configForwardSoftLimitThreshold(RobotMap.kHoodMaxAngle * RobotMap.kHoodTicksPerDegree); //forward decreases the angle in this context, might change later (would also have to change setAngle)
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

        Utils.configMotionMagic(m_hood, 1024, 1024); //TODO: determine whether velocity/accel param are arbitrary

        m_hood.setNeutralMode(NeutralMode.Brake);
    }

    public synchronized void setAngle(double angleDeg){
        //angle starts at 45 deg (from horizontal) and largest change should go to 0 deg
        setBrakeMode(angleDeg != 0);
        m_hood.set(ControlMode.MotionMagic, (RobotMap.kHoodMaxAngle - angleDeg) * RobotMap.kHoodTicksPerDegree);
    }

    /**
     *
     * @return the current target angle in degrees
     */
    public double getAngleSetpoint(){
        return RobotMap.kHoodMaxAngle - (m_hood.getClosedLoopTarget() / RobotMap.kHoodTicksPerDegree);
    }

    public double getRelativeHoodAngle(){
        return RobotMap.kHoodMaxAngle - (m_hood.getSelectedSensorPosition() / RobotMap.kHoodTicksPerDegree);
    }

    public boolean canShoot(){
        return(Math.abs(getAngleSetpoint() - getRelativeHoodAngle()) <= 2);
        //2 degrees of lenience
    }

    public void setBrakeMode(boolean state){
        if(state)
            {m_hood.setNeutralMode(NeutralMode.Brake);}
        else
            {m_hood.setNeutralMode(NeutralMode.Coast);}

    }

    public double getShuffleboardInput(){
        return writeHoodAngle.getDouble(0);
    }

    @Override
    public void outputToShuffleboard(){
        readHoodAngle.setNumber(getRelativeHoodAngle());

    }

    @Override
    public void teleopInit(){
        setBrakeMode(true);
    }

    @Override
    public void disable() {
        setBrakeMode(true);
    }
}


