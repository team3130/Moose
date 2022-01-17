package frc.robot.subsystems;

import com.ctre.phoenix.Util;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {
    private WPI_TalonFX m_motor;

    private ShuffleboardTab tab = Shuffleboard.getTab("Motor");

    private NetworkTableEntry sped = tab.add("Speed", 0.5).getEntry();
    private NetworkTableEntry RPM = tab.add("RPM", 0).getEntry();

    //Create and define all standard data types needed
    public Shooter() {
        m_motor = new WPI_TalonFX(RobotMap.CAN_SHOOTER_MOTOR);
    }

    public void spinMotor(double speed) {
        m_motor.set(speed);
    }

    public double getRawSpeed() {
        return m_motor.getSelectedSensorVelocity();
    }

    public double getRPM(){
        return Util.scaleNativeUnitsToRpm(RobotMap.kFlywheelRPMtoNativeUnitsScalar, (long) getRawSpeed());
    }

    public double getSpeedFromShuffleboard() {
        return sped.getDouble(0.5);
    }

    public void writeOutput() {
        RPM.setNumber(getRPM());
    }

    @Override
    public void periodic() {
        writeOutput();
    }
}
