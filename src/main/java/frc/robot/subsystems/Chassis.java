package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.sensors.Navx;

public class Chassis extends SubsystemBase {

// Any variables/fields used in the constructor must appear before the "INSTANCE" variable
// so that they are initialized before the constructor is called.

    //Create necessary objects
    private Navx m_navx = Navx.GetInstance();

    private WPI_TalonSRX m_rightFront, m_leftFront;

    private static final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

    private DifferentialDriveOdometry m_odometry;
    private DifferentialDriveKinematics m_kinematics;

    private DifferentialDrive m_drive;

    public void drive() {

    }

    //Create and define all standard data types needed
    public Chassis() {
        m_rightFront = new WPI_TalonSRX(RobotMap.CAN_CHASSIS_MOTOR_FRONTR);
        m_leftFront = new WPI_TalonSRX(RobotMap.CAN_CHASSIS_MOTOR_FRONTL);
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.

        m_rightFront.configFactoryDefault();
        m_leftFront.configFactoryDefault();

        m_drive = new DifferentialDrive(m_rightFront, m_leftFront);

        m_kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(28));
        m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

        configureBreakMode(true);
    }

    public void driveTank(double moveL, double moveR, boolean squaredInputs) {
        m_drive.tankDrive(moveL, moveR, squaredInputs);
    }

    public void configureBreakMode(boolean bool) {
        if (bool) {
            m_leftFront.setNeutralMode(NeutralMode.Brake);
            m_rightFront.setNeutralMode(NeutralMode.Brake);
        }
        else {
            m_leftFront.setNeutralMode(NeutralMode.Coast);
            m_rightFront.setNeutralMode(NeutralMode.Coast);
        }
    }

    public void driveArcade(double moveThrottle, double turnThrottle, boolean squaredInputs) {
        m_drive.arcadeDrive(moveThrottle, turnThrottle, squaredInputs);


    }
}

