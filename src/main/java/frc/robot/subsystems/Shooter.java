package frc.robot.subsystems; import com.ctre.phoenix.Util; import com.ctre.phoenix.motorcontrol.ControlMode; import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX; import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX; import edu.wpi.first.networktables.NetworkTableEntry; import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard; import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab; import edu.wpi.first.wpilibj2.command.SubsystemBase; import frc.robot.RobotMap; import static frc.robot.utils.Utils.configPIDF;public class Shooter extends SubsystemBase {private WPI_TalonFX m_flywheel; private WPI_TalonSRX m_indexer; private ShuffleboardTab tab = Shuffleboard.getTab("Motor"); private NetworkTableEntry sped = tab.add("Speed in RPM", 3000).getEntry(); private NetworkTableEntry RPM = tab.add("RPM", 0).getEntry(); private NetworkTableEntry shooterVoltageOut = tab.add("Shooter Voltage", 0).getEntry(); private NetworkTableEntry indexerReadPercent = tab.add("IndexerRead%", 0.5).getEntry(); private NetworkTableEntry indexerVoltageOut = tab.add("Indexer Voltage", 0).getEntry(); public Shooter() {m_flywheel = new WPI_TalonFX(RobotMap.CAN_SHOOTER_MOTOR); configPIDF(m_flywheel, RobotMap.kFlywheelP, RobotMap.kFlywheelI, RobotMap.kFlywheelD, RobotMap.kFlywheelF); m_indexer = new WPI_TalonSRX(RobotMap.CAN_INDEXER);} public void spinMotor(double speed) {m_flywheel.set(speed);} public double getRawSpeed() {return m_flywheel.getSelectedSensorVelocity();} public double getRPM(){return Util.scaleNativeUnitsToRpm(RobotMap.kFlywheelRPMtoNativeUnitsScalar, (long) getRawSpeed());} public double getSpeedFromShuffleboard() {return sped.getDouble(3000);}

    public void writeOutput() {
        RPM.setNumber(getRPM());
        shooterVoltageOut.setNumber(m_flywheel.getMotorOutputVoltage());
        indexerVoltageOut.setNumber(m_indexer.getMotorOutputVoltage());
    }


    public void setIndexerPercent(double percent){
        m_indexer.set(ControlMode.PercentOutput, percent);
    }

    public double getIndexerPercentFromShuffleboard() {
        return indexerReadPercent.getDouble(0.5);
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
        m_flywheel.set(ControlMode.PercentOutput, spin);
    }

    public void stop() {
        setOpenLoop(0.0);
    }

    public void setSpeed(double rpm) {
//        configPIDF(m_flywheelMaster, testP.getDouble(RobotMap.kFlywheelP), 0.0, testD.getDouble(RobotMap.kFlywheelD), RobotMap.kFlywheelF);
//        System.out.println("P: " + testP.getDouble(RobotMap.kFlywheelP) + " D: " + testD.getDouble(RobotMap.kFlywheelD) + " Setpoint: " + Util.scaleVelocityToNativeUnits(RobotMap.kFlywheelRPMtoNativeUnitsScalar, rpm));
        m_flywheel.set(ControlMode.Velocity, Util.scaleVelocityToNativeUnits(RobotMap.kFlywheelRPMtoNativeUnitsScalar, rpm));
        m_flywheel.getSelectedSensorPosition();
    }
}
