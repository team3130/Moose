package frc.robot.commands.Chassis;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotMap;
import frc.robot.SupportingClassess.Chooser;
import frc.robot.SupportingClassess.KugelCommandGroup;
import frc.robot.sensors.vision.Limelight;
import frc.robot.subsystems.Chassis;

import java.util.List;

public class FaceTarget extends KugelCommandGroup {
    // defining an instance to be used throughout the command and to be instantiated in the constructor of type parameter
    private final Chassis m_chassis;
    private final Limelight m_limelight;
    private final Chooser m_chooser;

    public FaceTarget(Chassis chassis, Limelight limelight, Chooser chooser) {
        //mapping to object passed through parameter
        m_chassis = chassis;
        m_requirements.add(chassis);
        m_limelight = limelight;
        m_chooser = chooser;
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        m_chassis.configRampRate(RobotMap.kMaxRampRate);
        super.clear();
        m_chassis.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                List.of(
                        m_chassis.getPose(),
                        new Pose2d(m_chassis.getPose().getX(), m_chassis.getPose().getY(), new Rotation2d(Math.toRadians(m_chassis.getAngle()) - m_limelight.getHeading().getRadians()))
                ),
                m_chooser.getConfig());
        // the object lives but gets re-scheduled and because of this,
        // this will add another command to the sequential command for
        // the whole of its life (effectively adding one path every
        // time this command is called) my horrible solution is to
        // make another object every time the button is pressed
        // and hope and pray that it works
        super.addCommands(m_chooser.getRamseteCommandFactory().apply(trajectory));
        super.initialize();
    }
}
