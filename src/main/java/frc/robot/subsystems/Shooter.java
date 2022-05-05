package frc.robot.subsystems;

import com.ctre.phoenix.Util;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.SupportingClassess.GeneralUtils;
import frc.robot.sensors.vision.Limelight;
import frc.robot.sensors.vision.WheelSpeedCalculations;
import frc.robot.utils.Utils;

public class Shooter extends SubsystemBase implements GeneralUtils {
    private final WPI_TalonFX m_flywheel;
    private final WPI_TalonFX m_flywheelFollower;
    private final WPI_TalonSRX m_indexer;

    private final Limelight m_limelight;

    private double flywheelSetSpeed = 3200; // default 3200 (3500 temp for Ben/Cody)
   // private double hoodWheelSetSpeed = 0;
    private double indexerSetSpeed = 0.5; // default 50%

    private final DigitalInput breakbeam;

    private final WheelSpeedCalculations shooterCurve;

    private final ShuffleboardTab tab = Shuffleboard.getTab("Shooter");

    private final NetworkTableEntry sped = tab.add("Shooter Set RPM", flywheelSetSpeed).getEntry();
    private final NetworkTableEntry followRPM = tab.add("Follower RPM", 0).getEntry();
    private final NetworkTableEntry RPM = tab.add("Shooter Current RPM", 0).getEntry();

    //private final NetworkTableEntry spedHoodWheel = tab.add("Shooter Top Set RPM", hoodWheelSetSpeed).getEntry();

    private final NetworkTableEntry P = tab.add("Top Flywheel P", RobotMap.kFlywheelP).getEntry();
    private final NetworkTableEntry I = tab.add("Top Flywheel I", RobotMap.kFlywheelI).getEntry();
    private final NetworkTableEntry D = tab.add("Top Flywheel D", RobotMap.kFlywheelD).getEntry();
    private final NetworkTableEntry V = tab.add("Top Flywheel V", RobotMap.flyWheelkV).getEntry();

    private final NetworkTableEntry time = tab.add("Time until Speed", 0).getEntry();
    private final NetworkTableEntry canShootManual = tab.add("Setpoint difference", 0).getEntry();

    private double accelTime = 0;


    //Create and define all standard data types needed
    public Shooter(Limelight limelight, WheelSpeedCalculations wheelSpeedCalculations) {
        m_flywheel = new WPI_TalonFX(RobotMap.CAN_SHOOTER_LEFT);
        m_flywheel.setInverted(false);
        
        m_flywheelFollower = new WPI_TalonFX(RobotMap.CAN_SHOOTER_RIGHT);
        m_flywheelFollower.setInverted(true);

        // restricting voltage for the flywheel
        m_flywheel.configVoltageCompSaturation(9);
        m_flywheel.enableVoltageCompensation(true);

        // restricting voltage for the hood wheel
        m_flywheelFollower.configVoltageCompSaturation(9);
        m_flywheelFollower.enableVoltageCompensation(true);

        m_flywheelFollower.follow(m_flywheel);

        Utils.configPIDF(m_flywheel, RobotMap.kFlywheelP, RobotMap.kFlywheelI, RobotMap.kFlywheelD, RobotMap.flyWheelkV);
        breakbeam = new DigitalInput(1);
        m_indexer = new WPI_TalonSRX(RobotMap.CAN_INDEXER);
        m_indexer.setNeutralMode(NeutralMode.Brake);
        m_indexer.setInverted(true);

        shooterCurve = wheelSpeedCalculations;

        m_limelight = limelight;
    }

    public void setAccelTime(double in){
        accelTime = in;
    }
    public boolean hasNards() {
        return !breakbeam.get();
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

   /* public double getHoodWheelSpeedFromShuffleboard() {
        return spedHoodWheel.getDouble(hoodWheelSetSpeed);
    }*/

    public void outputToShuffleboard() {
        RPM.setNumber(getRPM());
        time.setNumber(accelTime);
        canShootManual.setNumber( Math.abs(getRPM() - getSpeedFromShuffleboard()));
//        RPMHoodWheel.setNumber(getRPMHoodWheel());
//        shooterVoltageOut.setNumber(m_flywheel.getMotorOutputVoltage());
        SmartDashboard.putBoolean("Can Shoot?", canShoot());
        SmartDashboard.putBoolean("Break Beam", hasNards());
//        indexerVoltageOut.setNumber(m_indexer.getMotorOutputVoltage());
        // pidFlywheel.setPID(P.getDouble(RobotMap.kFlywheelP), I.getDouble(RobotMap.kFlywheelI), D.getDouble(RobotMap.kFlywheelD));
//        SmartDashboard.putBoolean("break beam", breakbeam.get());
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
        if (rpm != flywheelSetSpeed) {
            m_flywheel.set(ControlMode.Velocity, Util.scaleVelocityToNativeUnits(RobotMap.kFlywheelRPMtoNativeUnitsScalar, rpm));
            flywheelSetSpeed = rpm;
        }
    }

    public void stopAll() {
        m_indexer.set(ControlMode.PercentOutput, 0);
        m_flywheel.set(ControlMode.PercentOutput, 0);
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

    /*public void setHoodWheelSetSpeed(double speed) {
        hoodWheelSetSpeed = speed;
    }*/

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
        if (m_indexer.getMotorOutputPercent() != indexerSetSpeed) {
            setIndexerPercent(indexerSetSpeed);
        }
    }

    public boolean canShoot() {
        return Math.abs(getRPM() - flywheelSetSpeed) <= 50 && hasNards();
    }

    public boolean canShootSetFlywheel(double point) {
        return Math.abs(getRPM() - point) <= 50 && hasNards();
    }

    public WheelSpeedCalculations getShooterCurve(){
        return shooterCurve;
    }

    public void updatePID() {
        Utils.configPIDF(m_flywheel, P.getDouble(RobotMap.kFlywheelP), I.getDouble(RobotMap.kFlywheelI), D.getDouble(RobotMap.kFlywheelD), V.getDouble(RobotMap.flyWheelkV));
    }
}
