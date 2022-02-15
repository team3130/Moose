package frc.robot.subsystems;

import com.ctre.phoenix.Util;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

import static frc.robot.util.Utils.configPIDF;

public class Shooter extends SubsystemBase {
    private WPI_TalonFX m_motor;

    private ShuffleboardTab tab = Shuffleboard.getTab("Motor");

    private NetworkTableEntry sped = tab.add("Falcon Set Speed in RPM", 3000).getEntry();
    private NetworkTableEntry RPM = tab.add("Falcon RPM", 0).getEntry();

    //Create and define all standard data types needed
    public Shooter() {
        m_motor = new WPI_TalonFX(RobotMap.CAN_SHOOTER_MOTOR);

        configPIDF(m_motor,
                RobotMap.kFlywheelP,
                RobotMap.kFlywheelI,
                RobotMap.kFlywheelD,
                RobotMap.kFlywheelF);

        m_motor.setInverted(true);
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
        return sped.getDouble(3000);
    }

    public void writeOutput() {
        RPM.setNumber(getRPM());
    }

    @Override
    public void periodic() {
        writeOutput();
    }

    /**
     * Spin the turret flywheel at a raw percent VBus value
     *
     * @param spin percent of max voltage output
     */
    public void setOpenLoop(double spin) {
        m_motor.set(ControlMode.PercentOutput, spin);
    }

    public void stop() {
        setOpenLoop(0.0);
    }

    public void setSpeed(double rpm) {
//        configPIDF(m_flywheelMaster, testP.getDouble(RobotMap.kFlywheelP), 0.0, testD.getDouble(RobotMap.kFlywheelD), RobotMap.kFlywheelF);
//        System.out.println("P: " + testP.getDouble(RobotMap.kFlywheelP) + " D: " + testD.getDouble(RobotMap.kFlywheelD) + " Setpoint: " + Util.scaleVelocityToNativeUnits(RobotMap.kFlywheelRPMtoNativeUnitsScalar, rpm));

        m_motor.set(ControlMode.Velocity, Util.scaleVelocityToNativeUnits(RobotMap.kFlywheelRPMtoNativeUnitsScalar, rpm));
    }
}
