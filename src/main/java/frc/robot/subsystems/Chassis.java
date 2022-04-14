package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
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
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.SupportingClassess.GeneralUtils;
import frc.robot.commands.Chassis.DefaultDrive;
import frc.robot.sensors.Navx;

import java.util.Map;

public class Chassis extends SubsystemBase implements GeneralUtils {

    // Any variables/fields used in the constructor must appear before the
    // "INSTANCE" variable
    // so that they are initialized before the constructor is called.

    // Create necessary objects
    private Navx m_navx = Navx.GetInstance();

    private final WPI_TalonFX m_rightMotorFront;
    private final WPI_TalonFX m_rightMotorBack;
    private final WPI_TalonFX m_leftMotorFront;
    private final WPI_TalonFX m_leftMotorBack;

    private final Solenoid m_shifter; // true is low gear, false is high gear

    private final MotorControllerGroup m_motorsRight;
    private final MotorControllerGroup m_motorsLeft;

    private final DifferentialDriveOdometry m_odometry;
    private final DifferentialDriveKinematics m_kinematics;

    private final DifferentialDrive m_drive;

    private final SimpleMotorFeedforward m_feedforward;
    private final PIDController m_leftPIDController;
    private final PIDController m_rightPIDConttroller;

    private final Field2d m_fieldPos;

    private final PIDController m_spinnyPID;
    private final PIDController m_LaterallPID;

    // Network table pid stuff
    private ShuffleboardTab tab = Shuffleboard.getTab("Chassis");

    private NetworkTableEntry sliderMove = tab
            .add("Move Speed Sensitivity", 10)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 10))
            .getEntry();

    private NetworkTableEntry sliderRampRate = tab.add("Max ramp rate", RobotMap.kMaxRampRate)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0.1, "max", 3))
            .getEntry();

    private NetworkTableEntry sliderForwardVelocity = tab.add("Max forward velocity", RobotMap.kMaxHighGearDriveSpeed)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 1))
            .getEntry();

    private NetworkTableEntry sliderTurn = tab.add("Max turn speed", RobotMap.kMaxTurnThrottle)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 1))
            .getEntry();

    private NetworkTableEntry sliderSpin = tab.add("Spinny speed", RobotMap.kSpinnySpeed)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -1, "max", 1))
            .getEntry();


    private NetworkTableEntry Ps = tab.add("Chassis P",  RobotMap.ChassisSpinKP).getEntry();
    private NetworkTableEntry Is = tab.add("Chassis I", RobotMap.ChassisSpinKI).getEntry();
    private NetworkTableEntry Ds = tab.add("Chassis D", RobotMap.ChassisSpinKD).getEntry();

    private NetworkTableEntry Pl = tab.add("Chassis lateral P",  RobotMap.ChassisLateralP).getEntry();
    private NetworkTableEntry Il = tab.add("Chassis lateral I", RobotMap.ChassisLateralI).getEntry();
    private NetworkTableEntry Dl = tab.add("Chassis lateral D", RobotMap.ChassisLateralD).getEntry();

    // Create and define all standard data types needed
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

        m_rightMotorFront.configVoltageCompSaturation(RobotMap.kChassisMaxVoltage);
        m_leftMotorFront.configVoltageCompSaturation(RobotMap.kChassisMaxVoltage);
        m_rightMotorBack.configVoltageCompSaturation(RobotMap.kChassisMaxVoltage);
        m_leftMotorBack.configVoltageCompSaturation(RobotMap.kChassisMaxVoltage);
        m_rightMotorFront.enableVoltageCompensation(true);
        m_leftMotorFront.enableVoltageCompensation(true);
        m_rightMotorBack.enableVoltageCompensation(true);
        m_leftMotorBack.enableVoltageCompensation(true);

        m_rightMotorFront.setInverted(true);
        m_rightMotorBack.setInverted(true);

        configureBreakMode(true);

        // configure the motor groups
        m_motorsRight = new MotorControllerGroup(m_rightMotorFront, m_rightMotorBack);
        m_motorsLeft = new MotorControllerGroup(m_leftMotorFront, m_leftMotorBack);

        // making a differential drive object that includes the two motor groups
        m_drive = new DifferentialDrive(m_motorsLeft, m_motorsRight);
        m_drive.setDeadband(RobotMap.kDriveDeadband);
        m_drive.setSafetyEnabled(false);

        m_feedforward = new SimpleMotorFeedforward(RobotMap.ChassiskS, RobotMap.ChassiskV, RobotMap.ChassiskA);
        m_leftPIDController = new PIDController(RobotMap.LChassiskP, RobotMap.LChassiskI, RobotMap.LChassiskD);
        m_rightPIDConttroller = new PIDController(RobotMap.RChassiskP, RobotMap.RChassiskI, RobotMap.RChassiskD);

        // kinematics and odometry
        m_kinematics = new DifferentialDriveKinematics(RobotMap.trackDistance);
        m_odometry = new DifferentialDriveOdometry(new Rotation2d(Units.degreesToRadians(Navx.getAngle())));

        m_shifter = new Solenoid(RobotMap.CAN_PNMMODULE, PneumaticsModuleType.CTREPCM, RobotMap.PNM_Shift);
        m_shifter.set(false);

/*        m_leftMotorBack.follow(m_leftMotorFront);
        m_rightMotorBack.follow(m_rightMotorFront);*/
        m_fieldPos = new Field2d();

        m_spinnyPID = new PIDController(RobotMap.ChassisSpinKP, RobotMap.ChassisSpinKI, RobotMap.ChassisSpinKD);
        m_LaterallPID = new PIDController(RobotMap.ChassisLateralP, RobotMap.ChassisLateralI, RobotMap.ChassisLateralD);

        tuneTolerance();
    }

    public void driveTank(double moveL, double moveR, boolean squaredInputs) {
        m_drive.tankDrive(moveL, moveR, squaredInputs);
    }

    /**
     * configures the motors for brake or coast mode
     * an important note about break mode is that is harder on the bot
     * however if you do not have it on the bot could run into a car (yes this has
     * happened before)
     * 
     * @param brake whether to brake mode or coast mode
     */
    public void configureBreakMode(boolean brake) {
        if (brake) {
            m_leftMotorFront.setNeutralMode(NeutralMode.Brake);
            m_rightMotorFront.setNeutralMode(NeutralMode.Brake);
            m_leftMotorBack.setNeutralMode(NeutralMode.Brake);
            m_rightMotorBack.setNeutralMode(NeutralMode.Brake);
        } else {
            m_leftMotorFront.setNeutralMode(NeutralMode.Coast);
            m_rightMotorFront.setNeutralMode(NeutralMode.Coast);
            m_leftMotorBack.setNeutralMode(NeutralMode.Coast);
            m_rightMotorBack.setNeutralMode(NeutralMode.Coast);
        }
    }

    public double motorTemp() {
        double temp ;
        double temp2;
        if (m_leftMotorFront.getTemperature() >= m_leftMotorBack.getTemperature()) {
             temp = m_leftMotorFront.getTemperature();
        }
        else {
             temp = m_leftMotorBack.getTemperature();
        }
        if (m_rightMotorFront.getTemperature() >= m_rightMotorBack.getTemperature()) {
             temp2 = m_rightMotorFront.getTemperature();
        }
        else {
             temp2 = m_rightMotorBack.getTemperature();
        }

        if (temp >= temp2) {
            return temp;
        }
        else {
            return temp2;
        }
    }

    /**
     * Gets absolute distance traveled by the left side of the robot in high gear
     *
     * @return The absolute distance of the left side in meters
     */
    private double getDistanceL() {
        return m_leftMotorFront.getSelectedSensorPosition() / RobotMap.kEncoderResolution
                * (RobotMap.kChassisGearRatio) * ((RobotMap.kWheelDiameter) * Math.PI) * RobotMap.kChassisEncoderError;
    }

    /**
     * Gets absolute distance traveled by the right side of the robot in high gear
     *
     * @return The absolute distance of the right side in meters
     */
    private double getDistanceR() {
        return m_rightMotorFront.getSelectedSensorPosition() / RobotMap.kEncoderResolution
                * (RobotMap.kChassisGearRatio) * ((RobotMap.kWheelDiameter) * Math.PI) * RobotMap.kChassisEncoderError;
    }


    /**
     * Returns the current speed of the front left motor in high gear
     *
     * @return Current speed of the front left motor (meters per second)
     */
    public double getSpeedL() {
        return (m_leftMotorFront.getSelectedSensorVelocity() / RobotMap.kEncoderResolution
                * (RobotMap.kChassisGearRatio) * (Math.PI * RobotMap.kWheelDiameter)) * 10;
    }

    /**
     * Returns the current speed of the front right motor in high gear
     *
     * @return Current speed of the front right motor (meters per second)
     */
    public double getSpeedR() {
        return (m_rightMotorFront.getSelectedSensorVelocity() / RobotMap.kEncoderResolution
                * (RobotMap.kChassisGearRatio) * (Math.PI * RobotMap.kWheelDiameter)) * 10;
    }

    /**
     * Loops while the bot is in tele-op / this is a default command with the
     * scheduler running
     */
    @Override
    public void periodic() {
        // update odometry for relevant positional data
        m_odometry.update(Navx.getRotation(), getDistanceR(), getDistanceL());
        //updateRampThings();
    }

    /**
     * The default method for driving (used in
     * {@link DefaultDrive})
     * Drive in tank drive
     * 
     * @param moveThrottle  Left input throttle
     * @param turnThrottle  Right input throttle
     * @param squaredInputs whether to square inputs
     */
    public void driveArcade(double moveThrottle, double turnThrottle, boolean squaredInputs) {
        m_drive.arcadeDrive(moveThrottle, turnThrottle, squaredInputs);
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    private void resetEncoders() {
        m_leftMotorFront.setSelectedSensorPosition(0);
        m_rightMotorFront.setSelectedSensorPosition(0);
    }

    /**
     * resets odometry, to be called in rare situations
     * 
     * @param pose the pose to set odometry to
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        Navx.resetNavX();
        // sets 0 to be to the right of the bot, because 0 radians on the unit circle is to the right of north (as a bearing)
        m_odometry.resetPosition(pose, new Rotation2d(0));
    }

    /**
     * Returns the current speed of the robot by averaging the front left and right
     * motors
     *
     * @return Current speed of the robot
     */
    public double getSpeed() {
        return 0.5 * (getSpeedL() + getSpeedR());
    }

    public void updateRampThings() {
        RobotMap.kMaxRampRate = sliderRampRate.getDouble(RobotMap.kMaxRampRate);
        RobotMap.kMaxTurnThrottle = sliderTurn.getDouble(RobotMap.kMaxTurnThrottle);
        RobotMap.kMaxHighGearDriveSpeed = sliderForwardVelocity.getDouble(RobotMap.kMaxHighGearDriveSpeed);
        // configRampRate(RobotMap.kMaxRampRate);
    }

    /**
     * Configure the maximum ramping rate of the drivetrain while in Open Loop
     * control mode
     * <p>
     * Value of 0 disables ramping
     *
     * @param maxRampRateSeconds Minimum desired time to go from neutral to full
     *                           throttle
     */
    public void configRampRate(double maxRampRateSeconds) {
        m_rightMotorFront.configOpenloopRamp(maxRampRateSeconds);
        m_leftMotorFront.configOpenloopRamp(maxRampRateSeconds);
        m_rightMotorBack.configOpenloopRamp(maxRampRateSeconds);
        m_leftMotorBack.configOpenloopRamp(maxRampRateSeconds);
    }

    /**
     * checks if we have shifted
     * high gear is true low gear is false
     * 
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
     * gets the angle from navx and returns it unless navx is unplugged in which
     * case it just returns
     * 
     * @return the angle that navx reads
     */
    public double getAngle() {
        if (Navx.getNavxPresent()) {
            return Math.IEEEremainder(Navx.getAngle(), 360);
        } else {
            return 0;
        }
    }

    /**
     * Ramsete Command methods
     */

    /**
     * Returns the currently-estimated pose of the robot.
     * Used in Ramsete Command constructor param number 2
     * 
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Feeds the bot forward
     * Used in ramsete command constructor param number 4
     * 
     * @return {@link SimpleMotorFeedforward} object that is used in Ramsete command
     */
    public SimpleMotorFeedforward getFeedforward() {
        return m_feedforward;
    }

    /**
     * Gets the differential drive kinematics
     * Used in Ramsete Command constructor, param number 5
     * 
     * @return {@link DifferentialDrive} object for the bots kinematics
     */
    public DifferentialDriveKinematics getmKinematics() {
        return m_kinematics;
    }

    /**
     * Gets wheel speeds
     * Used in Ramsete Command Constructor, param num 6
     * 
     * @return wheel speeds as a {@link DifferentialDrive} object
     */
    public DifferentialDriveWheelSpeeds getSpeeds() {
        return new DifferentialDriveWheelSpeeds(getSpeedL(), getSpeedR());
    }

    /**
     * used in Ramsete Command constructor, param number 7
     * 
     * @return left pidcontroller
     */
    public PIDController getleftPIDController() {
        return m_leftPIDController;
    }

    /**
     * used in Ramsete Command constructor, param number 8
     * 
     * @return right pidcontroller
     */
    public PIDController getRightPIDController() {
        return m_rightPIDConttroller;
    }

    /**
     * Sets the motor voltage outputs and feeds the drivetrain forward
     * Used in Ramsete Command constructor, param number 9
     * 
     * @param leftVolts  voltage on the left side
     * @param rightVolts voltage on the right side
     */
    public void setOutput(double leftVolts, double rightVolts) {
        m_motorsLeft.setVoltage(leftVolts);
        m_motorsRight.setVoltage(rightVolts);
        m_drive.feed();
    }

    /**
     * Output values ot shuffleboard
     */
    public void outputToShuffleboard() {
        // current velocity
//        SmartDashboard.putNumber("Chassis Right Velocity", getSpeedR());
//        SmartDashboard.putNumber("Chassis Left Velocity", getSpeedL());

        // percent output of motors
//        SmartDashboard.putNumber("Chassis Right Output %", m_rightMotorFront.getMotorOutputPercent());
//        SmartDashboard.putNumber("Chassis Left Output %", m_leftMotorFront.getMotorOutputPercent());

        // shifting
//        SmartDashboard.putNumber("Chassis Distance R", getDistanceR());
//        SmartDashboard.putNumber("Chassis Distance L", getDistanceL());

        // bot position
        SmartDashboard.putNumber("Robot position X", m_odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Robot position Y", m_odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Robot rotation", m_odometry.getPoseMeters().getRotation().getDegrees());

        SmartDashboard.putBoolean("Shifted", m_shifter.get());

        SmartDashboard.putNumber("Chassis Spinny Position Error", m_spinnyPID.getPositionError());
        SmartDashboard.putNumber("Chassis Spinny Velocity Error", m_spinnyPID.getVelocityError());

        SmartDashboard.putNumber("Chassis Lateral Position Error", m_LaterallPID.getPositionError());
        SmartDashboard.putNumber("Chassis Lateral Velocity Error", m_LaterallPID.getVelocityError());

//        LBTemp.setNumber(m_leftMotorBack.getTemperature());
//        RBTemp.setNumber(m_rightMotorBack.getTemperature());
//        LFTemp.setNumber(m_leftMotorFront.getTemperature());
//        RFTemp.setNumber(m_rightMotorFront.getTemperature());

        m_fieldPos.setRobotPose(this.getPose());
        SmartDashboard.putData("Field position", m_fieldPos);

        RobotMap.kSpinnySpeed = sliderSpin.getDouble(RobotMap.kSpinnySpeed);
    }

    @Override
    public void teleopInit() {}

    @Override
    public void disable() {
        m_leftMotorFront.set(0);
        m_rightMotorFront.set(0);
        m_drive.stopMotor();
    }

    public double getMoveSpeedSensitivityFromShuffleboard() {
        return sliderMove.getDouble(10);
    }

    public double getTurnSpeedSensitivityFromShuffleboard() {
        return sliderTurn.getDouble(10);
    }

    public void faceTarget(double angle) {
        driveArcade(0, -m_spinnyPID.calculate(angle), false);
    }

    public void faceTarget(double angle, double translationPos) {
        driveArcade(m_LaterallPID.calculate(translationPos), -m_spinnyPID.calculate(angle), false);
    }

    public boolean getAtSetpoint() {
        return m_spinnyPID.atSetpoint();
    }

    public void setSpinnySetPoint(double setpoint) {
        System.out.println("Set point: " + setpoint);
        m_spinnyPID.setSetpoint(setpoint);
    }

    public void setLateralSetPoint(double setpoint) {
        m_LaterallPID.setSetpoint(setpoint);
    }

    public void updatePIDValues() {
        m_spinnyPID.setPID(Ps.getDouble(RobotMap.ChassisSpinKP), Is.getDouble(RobotMap.ChassisSpinKI), Ds.getDouble(RobotMap.ChassisSpinKD));
        m_LaterallPID.setPID(Pl.getDouble(RobotMap.ChassisSpinKP), Il.getDouble(RobotMap.ChassisSpinKI), Dl.getDouble(RobotMap.ChassisSpinKD));
    }

    public void tuneTolerance() {
        m_spinnyPID.setTolerance(3.5, 0.02);
        m_spinnyPID.setIntegratorRange(-0.1, 0.1);
        m_LaterallPID.setTolerance(0.25, 0.25);
    }

    public void resetPIDLoop() {
        m_spinnyPID.reset();
        m_LaterallPID.reset();
    }

    public double getCurrentVectorDist() {
        return getPose().getX();
    }

}
