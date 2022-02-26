package frc.robot.team3130;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.team3130.SupportingClasses.BallManager;
import frc.robot.team3130.SupportingClasses.Graph;
import frc.robot.team3130.commands.DefaultDrive;
import frc.robot.team3130.subsystems.Chassis;
import static edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/**
 * All objects that are going to be used that are instantiated once should be
 * defined and accessible here
 */
public class RobotContainer {
    // define subsystems here
    private final Chassis m_chassis = new Chassis();


    private final Graph m_graph = new Graph();
    private final BallManager m_ballManager = new BallManager(m_graph, m_chassis);

    // reminder that Singletons are deprecated, please do not use them even for
    // subsystems
    // EX: private final ExampleSubsystem m_exampleSubsystem = new
    // ExampleSubsystem();

    // make getters for subsystems here

    public RobotContainer() {
        defineButtonBindings();
        m_chassis.setDefaultCommand(new DefaultDrive(m_chassis));
    }

    // Joysticks
    public static Joystick m_driverGamepad = new Joystick(0);
    public static Joystick m_weaponsGamepad = new Joystick(1);

    private void defineButtonBindings() {
    }

    public void outputToShuffleBoard() {
        m_chassis.outputToShuffleboard();
    }

    public Command getTrajectory() {
        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                        RobotMap.ksVolts,
                        RobotMap.kvVoltSecondsPerMeter,
                        RobotMap.kaVoltSecondsSquaredPerMeter),
                RobotMap.kDriveKinematics,
                10);

        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
                RobotMap.kMaxSpeedMetersPerSecond,
                RobotMap.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(RobotMap.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint);

        Trajectory trajectory = m_ballManager.getPath(config);
        
        RamseteCommand ramseteCommand = new RamseteCommand(
                trajectory,
                m_chassis::getPose,
                new RamseteController(RobotMap.kRamseteB, RobotMap.kRamseteZeta),
                new SimpleMotorFeedforward(
                        RobotMap.ksVolts,
                        RobotMap.kvVoltSecondsPerMeter,
                        RobotMap.kaVoltSecondsSquaredPerMeter),
                RobotMap.kDriveKinematics,
                m_chassis::getSpeeds,
                new PIDController(RobotMap.kPDriveVel, 0, 0),
                new PIDController(RobotMap.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                m_chassis::setOutput,
                m_chassis);

        // Reset odometry to the starting pose of the trajectory.
        m_chassis.resetOdometry(trajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> m_chassis.setOutput(0, 0));
    }
}
