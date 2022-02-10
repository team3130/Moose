package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    // Network table pid stuff
    private ShuffleboardTab tab = Shuffleboard.getTab("Chassis");

    private NetworkTableEntry P =
            tab.add("Chassis P", .5).getEntry();
    private NetworkTableEntry I =
            tab.add("Chassis I", 0).getEntry();
    private NetworkTableEntry D =
            tab.add("Chassis D", 0).getEntry();

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

        configureBreakMode(true);

        m_rightMotorFront.configVoltageCompSaturation(RobotMap.kChassisMaxVoltage);
        m_leftMotorFront.configVoltageCompSaturation(RobotMap.kChassisMaxVoltage);
        m_rightMotorFront.enableVoltageCompensation(true);
        m_leftMotorFront.enableVoltageCompensation(true);

        //TODO: CHECK ALL OF THESE BEFORE PUTTING THE BOT ON THE GROUND
        m_rightMotorFront.setInverted(false);
        m_leftMotorFront.setInverted(false);
        m_rightMotorBack.setInverted(false);
        m_leftMotorBack.setInverted(false);

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
        m_odometry = new DifferentialDriveOdometry(new Rotation2d(Units.degreesToRadians(Navx.getAngle())));

        m_shifter = new Solenoid(RobotMap.CAN_PNMMODULE, PneumaticsModuleType.CTREPCM, RobotMap.PNM_Shift);
        m_shifter.set(false);
    }

    public void driveTank(double moveL, double moveR, boolean squaredInputs) {
        m_drive.tankDrive(moveL, moveR, squaredInputs);
    }

    /**
     * configures the motors for brake or coast mode
     * an important note about break mode is that is harder on the bot
     *      however if you do not have it on the bot could run into a car (yes this has happened before)
     * @param brake whether to brake mode or coast mode
     */
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
    /**
     * Gets absolute distance traveled by the right side of the robot in low gear
     *
     * @return The absolute distance of the right side in meters
     */
    private double getDistanceLowGearR() {
        return m_rightMotorFront.getSelectedSensorPosition() / RobotMap.kEncoderResolution * (1 / RobotMap.kChassisLowGearRatio) * ((RobotMap.kWheelDiameter) * Math.PI);
    }
    /**
     * Gets absolute distance traveled by the left side of the robot in low gear
     *
     * @return The absolute distance of the left side in meters
     */
    private double getDistanceLowGearL() {
        return m_leftMotorFront.getSelectedSensorPosition() / RobotMap.kEncoderResolution * (1 / RobotMap.kChassisLowGearRatio) * ((RobotMap.kWheelDiameter) * Math.PI);
    }
    /**
     * Gets absolute distance traveled by the left side of the robot in high gear
     *
     * @return The absolute distance of the left side in meters
     */
    private double getDistanceHighGearL() {
        return m_leftMotorFront.getSelectedSensorPosition() / RobotMap.kEncoderResolution * (1 / RobotMap.kChassisHighGearRatio) * ((RobotMap.kWheelDiameter) * Math.PI);
    }
    /**
     * Gets absolute distance traveled by the right side of the robot in high gear
     *
     * @return The absolute distance of the right side in meters
     */
    private double getDistanceHighGearR() {
        return m_rightMotorFront.getSelectedSensorPosition() / RobotMap.kEncoderResolution * (1 / RobotMap.kChassisHighGearRatio) * ((RobotMap.kWheelDiameter) * Math.PI);
    }

    /**
     * Returns the current speed of the front left motor in low gear
     *
     * @return Current speed of the front left motor (meters per second)
     */
    public double getSpeedLowGearL() {
        return (m_leftMotorFront.getSelectedSensorVelocity() / RobotMap.kEncoderResolution * (1 / RobotMap.kChassisLowGearRatio) * (Math.PI * RobotMap.kWheelDiameter))  * 10;
    }

    /**
     * Returns the current speed of the front right motor in low gear
     *
     * @return Current speed of the front right motor (meters per second)
     */
    public double getSpeedLowGearR() {
        return (m_rightMotorFront.getSelectedSensorVelocity() / RobotMap.kEncoderResolution * (1 / RobotMap.kChassisLowGearRatio) * (Math.PI * RobotMap.kWheelDiameter))  * 10;
    }

    /**
     * Returns the current speed of the front left motor in high gear
     *
     * @return Current speed of the front left motor (meters per second)
     */
    public double getSpeedHighGearL() {
        return (m_leftMotorFront.getSelectedSensorVelocity() / RobotMap.kEncoderResolution * (1 / RobotMap.kChassisHighGearRatio) * (Math.PI * RobotMap.kWheelDiameter))  * 10;
    }

    /**
     * Returns the current speed of the front right motor in high gear
     *
     * @return Current speed of the front right motor (meters per second)
     */
    public double getSpeedHighGearR() {
        return (m_rightMotorFront.getSelectedSensorVelocity() / RobotMap.kEncoderResolution * (1 / RobotMap.kChassisHighGearRatio) * (Math.PI * RobotMap.kWheelDiameter))  * 10;
    }

    /**
     * Loops while the bot is in tele-op / this is a default command with the scheduler running
     */
    @Override
    public void periodic() {
        // update odometry for relevant positional data
        if (!m_shifter.get()) {
            m_odometry.update(Navx.getRotation(), getDistanceLowGearL(), getDistanceLowGearR());
        }
        else {
            m_odometry.update(Navx.getRotation(), getDistanceHighGearL(), getDistanceHighGearR());
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
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public static double getHeading() {
        return Navx.getAngle();
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     * */
    private void resetEncoders() {
        m_leftMotorFront.setSelectedSensorPosition(0);
        m_rightMotorFront.setSelectedSensorPosition(0);
    }

    /**
     * resets odometry, to be called in rare situations
     * @param pose the pose to set odometry to
     */
    private void resetOdometry(Pose2d pose) {
        resetEncoders();
        Navx.resetNavX();
        m_odometry.resetPosition(pose, Navx.getRotation());
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

    /**
     * checks if we have shifted
     * high gear is true low gear is false
     * @return whether we have shifted or not
     */
    public boolean isShifted() {
        return m_shifter.get();
    }

    /**
     * Shifts the drivetrain gear box into an absolute gear
     *
     * @param shiftVal true is high gear, false is low gear
     */
    public void shift(boolean shiftVal) {
        m_shifter.set(shiftVal);
    }

    /**
     * Toggle shifting state
     */
    public void shift() {
        m_shifter.toggle();
    }

    /**
     * gets the angle from navx and returns it unless navx is unplugged in which case it just returns
     * @return the angle that navx reads
     */
    public double getAngle() {
        if (Navx.getNavxPresent()) {
            return Navx.getAngle();
        }
        else {
            return 0;
        }
    }

    /**
     * Ramsete Command methods
     */

    /**
     * Returns the currently-estimated pose of the robot.
     * Used in Ramsete Command constructor param number 2
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Feeds the bot forward
     * Used in ramsete command constructor param number 4
     * @return {@link SimpleMotorFeedforward} object that is used in Ramsete command
     */
    public SimpleMotorFeedforward getFeedforward() {
        return m_feedforward;
    }

    /**
     * Gets the differential drive kinematics
     * Used in Ramsete Command constructor, param number 5
     * @return {@link DifferentialDrive} object for the bots kinematics
     */
    public DifferentialDriveKinematics getmKinematics() {
        return m_kinematics;
    }

    /**
     * Gets wheel speeds
     * Used in Ramsete Command Constructor, param num 6
     * @return wheel speeds as a {@link DifferentialDrive} object
     */
    public DifferentialDriveWheelSpeeds getSpeeds() {
        if (m_shifter.get()) {
            return new DifferentialDriveWheelSpeeds(getSpeedHighGearL() / 10, getSpeedHighGearR() / 10);
        }
        else {
            return new DifferentialDriveWheelSpeeds(getSpeedLowGearL() / 10, getSpeedLowGearR() / 10);
        }
    }

    /**
     *  used in Ramsete Command constructor, param number 7
     * @return left pidcontroller
     */
    public PIDController getleftPIDController() {
        return m_leftPIDController;
    }

    /**
     *  used in Ramsete Command constructor, param number 8
     * @return right pidcontroller
     */
    public PIDController getRightPIDController() {
        return m_rightPIDConttroller;
    }

    /**
     * Sets the motor voltage outputs and feeds the drivetrain forward
     * Used in Ramsete Command constructor, param number 9
     * @param leftVolts voltage on the left side
     * @param rightVolts voltage on the right side
     */
    public void setOutput(double leftVolts, double rightVolts) {
        m_leftMotorFront.setVoltage(leftVolts);
        m_rightMotorFront.setVoltage(rightVolts);
        m_drive.feed();
    }

    /**
     * Output values ot shuffleboard
     */
    public void outputToShuffleboard() {
        // current velocity
        SmartDashboard.putNumber("Chassis Right Velocity", (isShifted()) ? getSpeedHighGearR() : getSpeedLowGearR());
        SmartDashboard.putNumber("Chassis Left Velocity", (isShifted()) ? getSpeedHighGearL() : getSpeedLowGearL());
        
        // percent output of motors
        SmartDashboard.putNumber("Chassis Right Output %", m_rightMotorFront.getMotorOutputPercent());
        SmartDashboard.putNumber("Chassis Left Output %", m_leftMotorFront.getMotorOutputPercent());

        // distances
        // TODO: switch to an encoder outside the gears cause distance is cringe with shifting
        SmartDashboard.putNumber("Chassis Distance R", (isShifted()) ? getDistanceHighGearR() : getDistanceLowGearR());
        SmartDashboard.putNumber("Chassis Distance L", (isShifted()) ? getDistanceHighGearL() : getDistanceLowGearL());

        // bot position
        SmartDashboard.putNumber("Robot position X", m_odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Robot position Y", m_odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Robot rotation", m_odometry.getPoseMeters().getRotation().getDegrees());

    }

}

