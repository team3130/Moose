package frc.robot.subsystems;

import com.ctre.phoenix.Util;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

import static frc.robot.utils.Utils.configPIDF;

public class Shooter extends SubsystemBase implements GeneralUtils {
    private WPI_TalonFX m_flywheel;
    private WPI_TalonSRX m_indexer;

    private double flywheelSetSpeed = 3200; // default 3200
    private double indexerSetSpeed = 0.5; // default 50%

    private ShuffleboardTab tab = Shuffleboard.getTab("Shooter");

    private NetworkTableEntry sped = tab.add("Shooter Write RPM", flywheelSetSpeed).getEntry();
    private NetworkTableEntry RPM = tab.add("Shooter Read RPM", 0).getEntry();
    private NetworkTableEntry shooterVoltageOut = tab.add("Shooter Voltage", 0).getEntry();

    private NetworkTableEntry indexerPercent = tab.add("IndexerWrite%", indexerSetSpeed).getEntry();
    private NetworkTableEntry indexerRPM = tab.add("Indexer Write RPM", indexerSetSpeed).getEntry();

    //Create and define all standard data types needed
    public Shooter() {
        m_flywheel = new WPI_TalonFX(RobotMap.CAN_SHOOTER_MOTOR);
        m_flywheel.setInverted(true);

        configPIDF(m_flywheel,
                RobotMap.kFlywheelP,
                RobotMap.kFlywheelI,
                RobotMap.kFlywheelD,
                RobotMap.kFlywheelF);
        m_indexer = new WPI_TalonSRX(RobotMap.CAN_INDEXER);
        m_indexer.setNeutralMode(NeutralMode.Coast);
        
    }



    public double getRawSpeed() {
        return m_flywheel.getSelectedSensorVelocity();
    }

    public double getRPM(){
        return Util.scaleNativeUnitsToRpm(RobotMap.kFlywheelRPMtoNativeUnitsScalar, (long) getRawSpeed());
    }

    public double getSpeedFromShuffleboard() {
        return sped.getDouble(flywheelSetSpeed);
    }

    public double getIndexerSpeedFromShuffleboard(){
        return indexerRPM.getDouble(indexerSetSpeed);
    }

    public void outputToShuffleboard() {
        RPM.setNumber(getRPM());
        shooterVoltageOut.setNumber(m_flywheel.getMotorOutputVoltage());
//        indexerVoltageOut.setNumber(m_indexer.getMotorOutputVoltage());
    }


    public void setIndexerPercent(double percent){
        m_indexer.set(ControlMode.PercentOutput, percent);
    }

    public double getIndexerPercentFromShuffleboard() {
        return indexerPercent.getDouble(indexerSetSpeed);
    }

    public void teleopInit() {
        flywheelSetSpeed = getSpeedFromShuffleboard();
    }

    @Override
    public void disable() {
        m_flywheel.set(ControlMode.PercentOutput, 0);
        m_indexer.set(ControlMode.PercentOutput, 0);
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

    public void setFlywheelSpeed(double rpm) {
//        configPIDF(m_flywheelMaster, testP.getDouble(RobotMap.kFlywheelP), 0.0, testD.getDouble(RobotMap.kFlywheelD), RobotMap.kFlywheelF);
//        System.out.println("P: " + testP.getDouble(RobotMap.kFlywheelP) + " D: " + testD.getDouble(RobotMap.kFlywheelD) + " Setpoint: " + Util.scaleVelocityToNativeUnits(RobotMap.kFlywheelRPMtoNativeUnitsScalar, rpm));
        m_flywheel.set(ControlMode.Velocity, Util.scaleVelocityToNativeUnits(RobotMap.kFlywheelRPMtoNativeUnitsScalar, rpm));
        m_flywheel.getSelectedSensorPosition();
    }

    public void setIndexerSpeed(double rpm){
        m_indexer.set(ControlMode.Velocity, Util.scaleVelocityToNativeUnits(RobotMap.kIndexerRPMtoNativeUnitsScalar, rpm));
        m_indexer.getSelectedSensorVelocity(); 

    }

    /**
     *
     * @return the current speed flywheel will be set at
     */
    public double getFlywheelSetSpeed() {
        return flywheelSetSpeed;
    }

    /**
     * set the flywheelSetSpeed variable
     * @param speed in rpm
     */
    public void setFlywheelSetSpeed(double speed) {
        flywheelSetSpeed = speed;
    }

    /**
     * Runs flywheel at flywheelSetSpeed
     */
    public void feedFlywheel() {
        setFlywheelSpeed(flywheelSetSpeed);
    }

    public double getIndexerSetSpeed() {
        return indexerSetSpeed;
    }

    public void setIndexerSetSpeed(double indexerSetSpeed) {
        this.indexerSetSpeed = indexerSetSpeed;
    }

    /**
     * Runs indexer at indexerSetSpeed
     */
    public void feedIndexer() {
        setIndexerPercent(indexerSetSpeed);
    }

    public boolean canShoot() {
        return getRPM() >= getSpeedFromShuffleboard() - 50; // 50 is the range
    }
}
