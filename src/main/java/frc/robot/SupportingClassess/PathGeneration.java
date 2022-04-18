package frc.robot.SupportingClassess;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.RobotMap;
import frc.robot.sensors.vision.Limelight;
import frc.robot.sensors.vision.WheelSpeedCalculations;
import frc.robot.subsystems.Chassis;

import java.util.List;
import java.util.function.Function;

public class PathGeneration {
    protected final double[] range;

    protected final Chassis m_chassis;
    protected final Limelight m_limelight;
    protected final Chooser m_chooser;

    protected final Function<Trajectory, RamseteCommand> ramseteCommandFactory;


    protected final TrajectoryConfig config;


    public PathGeneration(WheelSpeedCalculations wheelSpeedCalculations, Chassis chassis, Limelight limelight, Chooser chooser) {
        range = new double[] {
                wheelSpeedCalculations.getMainDataStorage().get(1).getDistance(),
                wheelSpeedCalculations.getMainDataStorage().get(wheelSpeedCalculations.getMainDataStorage().size() - 2).getDistance()
        };

        m_chassis = chassis;
        m_limelight = limelight;
        m_chooser = chooser;
        ramseteCommandFactory = chooser.getRamseteCommandFactory();
        config = chooser.getConfig();
    }

    public Trajectory getShootPath() {
        return getShootPath(m_chassis.getPose());
    }

    public Trajectory getShootPath(Pose2d startPose) {
        double distanceToTarget = (m_limelight.hasTrack()) ?
                m_limelight.getDistanceToTarget() :
                startPose.relativeTo(RobotMap.kTargetPose).getTranslation().getNorm();

        // if we are not in range
        if (!(distanceToTarget > range[0] && distanceToTarget < range[1])) {
            Rotation2d endingAngle = RobotMap.kTargetPose.relativeTo(startPose).getRotation();
            if (distanceToTarget > range[1]) {
                return TrajectoryGenerator.generateTrajectory(
                        List.of(
                            startPose,
                            new Pose2d(new Translation2d(distanceToTarget - range[1], endingAngle), endingAngle)
                        ),
                        config
                        );
            }
            // is this optional? if (distanceToTarget < range[0])
            else {
                return TrajectoryGenerator.generateTrajectory(
                        List.of(startPose,
                                new Pose2d(new Translation2d(range[0] - distanceToTarget, endingAngle), endingAngle)
                        ),
                        config
                );
            }
        }
        else {
            return null;
        }
    }


    public Trajectory getPathToBall(Pose2d startPose, Pose2d ball) {
        return TrajectoryGenerator.generateTrajectory(List.of(startPose, ball), m_chooser.getConfig());
    }

    public Trajectory getPathToBalls(Pose2d startPose, Pose2d ball1, Pose2d ball2) {
        return TrajectoryGenerator.generateTrajectory(List.of(startPose, ball1, ball2), m_chooser.getConfig());
    }
}
