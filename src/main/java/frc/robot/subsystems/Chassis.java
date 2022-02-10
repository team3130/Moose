package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.sensors.Navx;

public class Chassis extends SubsystemBase {

// Any variables/fields used in the constructor must appear before the "INSTANCE" variable
// so that they are initialized before the constructor is called.

    //Create necessary objects
    private Navx m_navx = Navx.GetInstance();

    private WPI_TalonFX m_rightMotorFront;
    private WPI_TalonFX m_rightMotorBack;
    private WPI_TalonFX m_leftMotorFront;
    private WPI_TalonFX m_leftMotorBack;

    private Solenoid m_shifter;

    private MotorControllerGroup m_motorsRight;
    private MotorControllerGroup m_motorsLeft;

    private DifferentialDriveOdometry m_odometry;
    private DifferentialDriveKinematics m_kinematics;

    private DifferentialDrive m_drive;

    private SimpleMotorFeedforward m_feedforward;
    private PIDController m_leftPIDController;
    private PIDController m_rightPIDConttroller;

    // The gyro sensor
    private static final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

    //Create and define all standard data types needed
    public Chassis() {
        // Making the motors
        m_rightMotorFront = new WPI_TalonFX(RobotMap.CAN_CHASSIS_MOTOR_FRONTR);
        m_leftMotorFront = new WPI_TalonFX(RobotMap.CAN_CHASSIS_MOTOR_FRONTL);
        m_rightMotorBack = new WPI_TalonFX(RobotMap.CAN_CHASSIS_MOTOR_BACKR);
        m_leftMotorBack = new WPI_TalonFX(RobotMap.CAN_CHASSIS_MOTOR_BACKL);

        // setting to factory default
        m_rightMotorFront.configFactoryDefault();
        m_leftMotorFront.configFactoryDefault();
        m_rightMotorBack.configFactoryDefault();
        m_leftMotorBack.configFactoryDefault();

        // configure the motor groups
        m_motorsRight = new MotorControllerGroup(m_rightMotorFront, m_rightMotorBack);
        m_motorsLeft = new MotorControllerGroup(m_leftMotorFront, m_leftMotorBack);

        // making a differential drive object that includes the two motor groups
        m_drive = new DifferentialDrive(m_motorsLeft, m_motorsRight);
        m_drive.setDeadband(RobotMap.kDriveDeadband);
        m_drive.setSafetyEnabled(false);

        m_feedforward = new SimpleMotorFeedforward(RobotMap.lowGearkS, RobotMap.lowGearkV,RobotMap.lowGearkA);
        m_leftPIDController = new PIDController(2.05, 0, 0);
        m_rightPIDConttroller = new PIDController(2.05, 0, 0);

        // kinematics and odometry
        m_kinematics = new DifferentialDriveKinematics(RobotMap.trackDistance);
        m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

        configureBreakMode(true);

        m_shifter = new Solenoid(RobotMap.CAN_PNMMODULE, PneumaticsModuleType.CTREPCM, RobotMap.PNM_Shift);
        m_shifter.set(false);

    }

    public void driveTank(double moveL, double moveR, boolean squaredInputs) {
        m_drive.tankDrive(moveL, moveR, squaredInputs);
    }

    public void configureBreakMode(boolean brake) {
        if (brake) {
            m_leftMotorFront.setNeutralMode(NeutralMode.Brake);
            m_rightMotorFront.setNeutralMode(NeutralMode.Brake);
            m_leftMotorBack.setNeutralMode(NeutralMode.Brake);
            m_rightMotorBack.setNeutralMode(NeutralMode.Brake);
        }
        else {
            m_leftMotorFront.setNeutralMode(NeutralMode.Coast);
            m_rightMotorFront.setNeutralMode(NeutralMode.Coast);
            m_leftMotorBack.setNeutralMode(NeutralMode.Coast);
            m_rightMotorBack.setNeutralMode(NeutralMode.Coast);
        }
    }

    private double getDistanceLowGearR() {
        return m_rightMotorFront.getSelectedSensorPosition() / RobotMap.kEncoderResolution * (1 / RobotMap.kChassisLowGearRatio) * ((RobotMap.kWheelDiameter) * Math.PI);
    }

    private double getDistanceLowGearL() {
        return m_leftMotorFront.getSelectedSensorPosition() / RobotMap.kEncoderResolution * (1 / RobotMap.kChassisLowGearRatio) * ((RobotMap.kWheelDiameter) * Math.PI);
    }

    private double getDistanceHighGearL() {
        return m_leftMotorFront.getSelectedSensorPosition() / RobotMap.kEncoderResolution * (1 / RobotMap.kChassisHighGearRatio) * ((RobotMap.kWheelDiameter) * Math.PI);
    }

    private double getDistanceHighGearR() {
        return m_rightMotorFront.getSelectedSensorPosition() / RobotMap.kEncoderResolution * (1 / RobotMap.kChassisHighGearRatio) * ((RobotMap.kWheelDiameter) * Math.PI);
    }

    public double getSpeedLowGearL() {
        return (m_leftMotorFront.getSelectedSensorVelocity() / RobotMap.kEncoderResolution * (1 / RobotMap.kChassisLowGearRatio) * (Math.PI * RobotMap.kWheelDiameter))  * 10;
    }

    public double getSpeedLowGearR() {
        return (m_rightMotorFront.getSelectedSensorVelocity() / RobotMap.kEncoderResolution * (1 / RobotMap.kChassisLowGearRatio) * (Math.PI * RobotMap.kWheelDiameter))  * 10;
    }

    public double getSpeedHighGearL() {
        return (m_leftMotorFront.getSelectedSensorVelocity() / RobotMap.kEncoderResolution * (1 / RobotMap.kChassisHighGearRatio) * (Math.PI * RobotMap.kWheelDiameter))  * 10;
    }

    public double getSpeedHighGearR() {
        return (m_rightMotorFront.getSelectedSensorVelocity() / RobotMap.kEncoderResolution * (1 / RobotMap.kChassisHighGearRatio) * (Math.PI * RobotMap.kWheelDiameter))  * 10;
    }

    @Override
    public void periodic() {
        // update odometry for relevant positional data
        if (!m_shifter.get()) {
            m_odometry.update(m_gyro.getRotation2d(), getDistanceLowGearL(), getDistanceLowGearR());
        }
        else {
            m_odometry.update(m_gyro.getRotation2d(), getDistanceHighGearL(), getDistanceHighGearR());
        }
    }

    /**
     * The default method for driving (used in {@link frc.robot.commands.DefaultDrive}
     * Drive in tank drive
     * @param moveThrottle Left input throttle
     * @param turnThrottle Right input throttle
     * @param squaredInputs whether to square inputs
     */
    public void driveArcade(double moveThrottle, double turnThrottle, boolean squaredInputs) {
        m_drive.arcadeDrive(moveThrottle, turnThrottle, squaredInputs);
    }

    /**
     * Tank drive but only giving volts
     * @param leftVolts volts on the left
     * @param rightVolts volts on the right
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        m_leftMotorFront.set(leftVolts);
        m_rightMotorFront.set(-rightVolts);
        m_drive.feed();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public static double getHeading() {
        return m_gyro.getRotation2d().getDegrees();
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        m_leftMotorFront.setSelectedSensorPosition(0);
        m_rightMotorFront.setSelectedSensorPosition(0);
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, m_gyro.getRotation2d());
    }

    /**
     * Returns the current speed of the robot by averaging the front left and right motors
     *
     * @return Current speed of the robot
     */
    public double getSpeed() {
        if (m_shifter.get()) {
            return 0.5 * (getSpeedHighGearL() + getSpeedHighGearR());
        }
        else {
            return 0.5 * (getSpeedLowGearL() + getSpeedLowGearR());
        }
    }

    /**
     * Configure the maximum ramping rate of the drivetrain while in Open Loop control mode
     * <p>
     * Value of 0 disables ramping
     *
     * @param maxRampRateSeconds Minimum desired time to go from neutral to full throttle
     */
    public void configRampRate(double maxRampRateSeconds) {
        m_rightMotorFront.configOpenloopRamp(maxRampRateSeconds);
        m_leftMotorFront.configOpenloopRamp(maxRampRateSeconds);
    }

    public DifferentialDriveWheelSpeeds getSpeeds() {
        if (m_shifter.get()) {
            return new DifferentialDriveWheelSpeeds(getSpeedHighGearL() / 10, getSpeedHighGearR() / 10);
        }
        else {
            return new DifferentialDriveWheelSpeeds(getSpeedLowGearL() / 10, getSpeedLowGearR() / 10);
        }
    }

    public boolean isShifted() {
        return m_shifter.get();
    }

    public double getAngle() {
        if (Navx.getNavxPresent()) {
            return Navx.getAngle();
        }
        else {
            return 0;
        }
    }


}

