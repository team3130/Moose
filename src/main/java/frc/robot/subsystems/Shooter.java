package frc.robot.subsystems;

import com.ctre.phoenix.Util;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.SupportingClassess.GeneralUtils;
import frc.robot.utils.Utils;

public class Shooter extends SubsystemBase implements GeneralUtils {
    private WPI_TalonFX m_flywheel;
    private WPI_TalonFX m_hoodWheel;
    private WPI_TalonSRX m_indexer;

    private double flywheelSetSpeed = 3200; // default 3200 (3500 temp for Ben/Cody)
    private double hoodWheelSetSpeed = 1000;
    private double indexerSetSpeed = 0.5; // default 50%

    private PIDController pidFlywheel, pidTopShooter;
    private SimpleMotorFeedforward pidFlyWheelF, pidTopShooterF;

    private ShuffleboardTab tab = Shuffleboard.getTab("Shooter");

    private NetworkTableEntry sped = tab.add("Shooter Set RPM", flywheelSetSpeed).getEntry();
    private NetworkTableEntry RPM = tab.add("Shooter Current RPM", 0).getEntry();
    private NetworkTableEntry shooterVoltageOut = tab.add("Shooter Voltage", 0).getEntry();

    private NetworkTableEntry spedHoodWheel = tab.add("Shooter Top Set RPM", hoodWheelSetSpeed).getEntry();
    private NetworkTableEntry RPMHoodWheel = tab.add("Shooter Top Current RPM", 0).getEntry();

    private NetworkTableEntry P = tab.add("Flywheel P", RobotMap.kFlywheelP).getEntry();
    private NetworkTableEntry I = tab.add("Flywheel I", RobotMap.kFlywheelI).getEntry();
    private NetworkTableEntry D = tab.add("Flywheel D", RobotMap.kFlywheelD).getEntry();


    //Create and define all standard data types needed
    public Shooter() {
        m_flywheel = new WPI_TalonFX(RobotMap.CAN_SHOOTER_MOTOR);
        m_flywheel.setInverted(false);

        m_hoodWheel = new WPI_TalonFX(RobotMap.CAN_SHOOTER_UPPER_MOTOR);
        m_hoodWheel.setInverted(false);

        Utils.configPIDF(m_flywheel, RobotMap.kFlywheelP, RobotMap.kFlywheelI, RobotMap.kFlywheelD, RobotMap.flyWheelkV);

        pidFlywheel = new PIDController(RobotMap.kFlywheelP, RobotMap.kFlywheelI, RobotMap.kFlywheelD);
        pidTopShooter = new PIDController(RobotMap.kFlywheelHoodP, RobotMap.kFlywheelHoodI, RobotMap.kFlywheelHoodD);

        pidFlyWheelF = new SimpleMotorFeedforward(RobotMap.flyWheelkS, RobotMap.flyWheelkV, RobotMap.flyWheelkA);
        pidTopShooterF = new SimpleMotorFeedforward(RobotMap.flyWheelkS, RobotMap.flyWheelkV, RobotMap.flyWheelkA);

        m_indexer = new WPI_TalonSRX(RobotMap.CAN_INDEXER);
        m_indexer.setNeutralMode(NeutralMode.Coast);
        m_indexer.setInverted(true);
    }

    public double getRawSpeed() {
        return m_flywheel.getSelectedSensorVelocity();
    }

    public double getRPM(){
        return Util.scaleNativeUnitsToRpm(RobotMap.kFlywheelRPMtoNativeUnitsScalar, (long) getRawSpeed());
    }

    public double getRPMHoodWheel() {
        return Util.scaleNativeUnitsToRpm(RobotMap.kTopShooterRPMToNativeUnitsScalar, (long) m_hoodWheel.getSelectedSensorVelocity());
    }

    public double getSpeedFromShuffleboard() {
        return sped.getDouble(flywheelSetSpeed);
    }

    public double getHoodWheelSpeedFromShuffleboard() {
        return spedHoodWheel.getDouble(hoodWheelSetSpeed);
    }

    public void outputToShuffleboard() {
        RPM.setNumber(getRPM());
        RPMHoodWheel.setNumber(getRPMHoodWheel());
        shooterVoltageOut.setNumber(m_flywheel.getMotorOutputVoltage());
//        indexerVoltageOut.setNumber(m_indexer.getMotorOutputVoltage());
        // pidFlywheel.setPID(P.getDouble(RobotMap.kFlywheelP), I.getDouble(RobotMap.kFlywheelI), D.getDouble(RobotMap.kFlywheelD));
    }

    public void setIndexerPercent(double percent){
        m_indexer.set(ControlMode.PercentOutput, percent);
    }

    public void teleopInit() {
        flywheelSetSpeed = getSpeedFromShuffleboard();
    }

    @Override
    public void disable() {
        m_flywheel.set(ControlMode.PercentOutput, 0);
        m_hoodWheel.set(ControlMode.PercentOutput, 0);
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
//        pidFlywheel.setSetpoint(rpm);
        m_flywheel.set(ControlMode.Velocity, Util.scaleVelocityToNativeUnits(RobotMap.kFlywheelRPMtoNativeUnitsScalar, rpm));
    }

    public void stopAll() {
        m_indexer.set(0);
        m_flywheel.set(0);
        m_hoodWheel.set(0);
    }

    public void setFlyWheelPIDLoop() {
/*        double out = pidFlywheel.calculate(Util.scaleNativeUnitsToRpm(RobotMap.kFlywheelRPMtoNativeUnitsScalar, (long) m_flywheel.getSelectedSensorVelocity())) + pidFlyWheelF.calculate(pidFlywheel.getSetpoint());
        System.out.println("flywheel velocity error: " + pidFlywheel.getVelocityError());
        m_flywheel.set(ControlMode.PercentOutput, out);*/
    }

    public void setHoodWheelTopSpeed(double rpm) {
        pidTopShooter.setSetpoint(rpm);
    }

    public void setHoodWheelPidLoop() {
        double out = pidTopShooter.calculate(Util.scaleNativeUnitsToRpm(RobotMap.kTopShooterRPMToNativeUnitsScalar, (long) m_hoodWheel.getSelectedSensorVelocity())) + pidTopShooterF.calculate(pidTopShooter.getSetpoint());
        System.out.println("Top shooter velocity error:" + pidTopShooter.getVelocityError());
        m_hoodWheel.set(ControlMode.PercentOutput, out);
    }

    public void spinHoodWheel() {
        m_hoodWheel.set(ControlMode.PercentOutput, 0.3);
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

    public void setHoodWheelSetSpeed(double speed) {
        hoodWheelSetSpeed = speed;
    }

    public void resetPID() {
        pidTopShooter.reset();
        pidFlywheel.reset();
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

    public void feedHoodWheel() {
        setHoodWheelTopSpeed(hoodWheelSetSpeed);
    }

}
