package frc.robot.subsystems;

import com.ctre.phoenix.Util;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.SupportingClassess.GeneralUtils;
import frc.robot.sensors.vision.WheelSpeedCalculations;
import frc.robot.utils.Utils;

public class Hood extends SubsystemBase implements GeneralUtils {

// Any variables/fields used in the constructor must appear before the "INSTANCE" variable
// so that they are initialized before the constructor is called.

    ShuffleboardTab tab = Shuffleboard.getTab("hood");

    NetworkTableEntry setPos = tab.add("Hood SetPos", 0).getEntry();

    NetworkTableEntry P = tab.add("Hood P", RobotMap.kHoodP).getEntry();
    NetworkTableEntry I = tab.add("Hood I", RobotMap.kHoodI).getEntry();
    NetworkTableEntry D = tab.add("Hood D", RobotMap.kHoodD).getEntry();
    NetworkTableEntry V = tab.add("Hood V", RobotMap.kHoodV).getEntry();

    //Create necessary objects
    private WPI_TalonSRX m_hood;
//    private WheelSpeedCalculations winchCurve;

    //Create and define all standard data types needed
    public Hood() {
        m_hood = new WPI_TalonSRX(RobotMap.CAN_HOOD_MOTOR);
        m_hood.setInverted(false);
//        winchCurve = new WheelSpeedCalculations(WheelSpeedCalculations.CurveMechanism.HOOD_WINCH);

        Utils.configMotionMagic(m_hood, 1024, 1024);
        Utils.configPIDF(m_hood,
                RobotMap.kHoodP,
                RobotMap.kHoodI,
                RobotMap.kHoodD,
                RobotMap.kHoodV);

        tab.add("Angle", 0);
    }

    public void setSpeed(double speed) {
        m_hood.set(speed);
    }

    public void zero() {
        m_hood.set(ControlMode.MotionMagic, 0);

    }

    public void toPos(double pos) {
        m_hood.set(ControlMode.MotionMagic, Util.scaleRotationsToNativeUnits(RobotMap.HoodScalarToRotations, pos));

    }

/*    public WheelSpeedCalculations getWinchCurve(){
        return winchCurve;
    }*/

    public double getDistance() {
        return Util.scaleNativeUnitsToRotations(RobotMap.HoodScalarToRotations, (long) m_hood.getSelectedSensorPosition());
    }

    public boolean canShoot(double currentSetpoint) {
        return Math.abs(currentSetpoint - m_hood.getSelectedSensorPosition()) <= 128;
    }

    @Override
    public void outputToShuffleboard() {
        SmartDashboard.putNumber("Angle", getDistance());
        SmartDashboard.putBoolean("Hood At Position", atPosition());
    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void disable() {

    }

    public boolean atPosition() {
        return m_hood.isMotionProfileFinished();
    }

    public double getSetPos() {
        return setPos.getDouble(0);
    }

    public void updatePID() {
        Utils.configPIDF(m_hood,
                P.getDouble(RobotMap.kHoodP),
                I.getDouble(RobotMap.kHoodI),
                D.getDouble(RobotMap.kHoodD),
                V.getDouble(RobotMap.kHoodV)
        );
    }
}

