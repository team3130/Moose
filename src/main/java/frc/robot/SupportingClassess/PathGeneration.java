package frc.robot.SupportingClassess;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotMap;
import frc.robot.commands.Intake.DeployAndSpintake;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.sensors.vision.Limelight;
import frc.robot.sensors.vision.WheelSpeedCalculations;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;

import java.util.List;
import java.util.function.Function;

public class PathGeneration {
    protected final double[] range;

    protected final Chassis m_chassis;
    protected final Limelight m_limelight;
    protected final Chooser m_chooser;

    protected final Function<Trajectory, RamseteCommand> ramseteCommandFactory;

    protected final Intake m_intake;
    protected final Magazine m_magazine;
    protected final Shooter m_shooter;

    protected final TrajectoryConfig config;


    public PathGeneration(WheelSpeedCalculations wheelSpeedCalculations, Chassis chassis, Limelight limelight, Chooser chooser, Intake intake, Magazine magazine, Shooter shooter) {
        range = new double[] {
                wheelSpeedCalculations.getMainDataStorage().get(1).getDistance(),
                wheelSpeedCalculations.getMainDataStorage().get(wheelSpeedCalculations.getMainDataStorage().size() - 2).getDistance()
        };

        m_chassis = chassis;
        m_limelight = limelight;
        m_chooser = chooser;
        m_intake = intake;
        m_magazine = magazine;
        m_shooter = shooter;
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

    public SequentialCommandGroup getCircut(Pose2d start, Ball ball1, Ball ball2) {
        double firstBallX = ball1.getX();
        double firstBallY = ball1.getY();

        double secondBallX = ball2.getX();
        double secondBallY = ball2.getY();

        Rotation2d rotationBetweenBall1AndBall2 =
                new Rotation2d(Math.atan2(secondBallY - firstBallY, secondBallX - firstBallX));

        Pose2d firstBall = new Pose2d(
                firstBallX,
                firstBallY,
                new Rotation2d((rotationBetweenBall1AndBall2.getRadians() + start.getRotation().getRadians()) / 2)
        );


        Rotation2d rotationBetweenSecondBallAndTarget =
                new Rotation2d(Math.atan2(RobotMap.kTargetPose.getY() - secondBallY, RobotMap.kTargetPose.getX() - secondBallX));

        Pose2d secondBall = new Pose2d(secondBallX, secondBallY, new Rotation2d((rotationBetweenSecondBallAndTarget.getRadians() + firstBall.getRotation().getRadians()) / 2.0));


        CommandBase deployAndSpintake = new DeployAndSpintake(m_intake, m_magazine, 1);

        RamseteCommand goToBalls = ramseteCommandFactory.apply(
                TrajectoryGenerator.generateTrajectory(
                        List.of(
                                start,
                                firstBall,
                                secondBall
                                ),
                        config
                )
        );

        RamseteCommand goToShoot = ramseteCommandFactory.apply(getShootPath(secondBall));

        CommandBase shoot = new Shoot(m_shooter, m_magazine, m_chassis, m_limelight);

        return new SequentialCommandGroup(
                new ParallelDeadlineGroup(goToBalls, deployAndSpintake),
                goToShoot,
                shoot);
    }

    public Trajectory getPathToBall(Pose2d startPose, Pose2d ball) {
        return TrajectoryGenerator.generateTrajectory(List.of(startPose, ball), m_chooser.getConfig());
    }

    public Trajectory getPathToBalls(Pose2d startPose, Pose2d ball1, Pose2d ball2) {
        return TrajectoryGenerator.generateTrajectory(List.of(startPose, ball1, ball2), m_chooser.getConfig());
    }
}
