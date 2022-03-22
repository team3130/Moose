package frc.robot.SupportingClassess;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.commands.Chassis.FaceTarget;
import frc.robot.commands.Intake.DeployAndSpintake;
import frc.robot.commands.Shooter.SetFlywheelRPM;
import frc.robot.subsystems.Chassis;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

public class Chooser {
    private final SendableChooser<AutonCommand> m_autonChooser;
    private final RobotContainer container;
    private final Function<Path, Trajectory> trajectoryFactory;
    private final Function<Trajectory, RamseteCommand> ramseteCommandFactory;
    private final Function<Trajectory, AutonCommand> autonCmdFactory;
    private final TrajectoryConfig config;
    private AutonCommand testPath;

    public Chooser(SendableChooser<AutonCommand> m_autonChooser, RobotContainer container) {
        this.m_autonChooser = m_autonChooser;
        this.container = container;

        Chassis chassis = container.getChassis();

        DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                chassis.getFeedforward(),
                chassis.getmKinematics(),
                12);

        config = new TrajectoryConfig(RobotMap.kMaxVelocityMPS, RobotMap.kMaxAccelerationMPS);
        config.setKinematics(container.getChassis().getmKinematics()).addConstraint(autoVoltageConstraint);

        // lambda to build trajectories
        trajectoryFactory =
                (Path path) -> {
                    Trajectory toReturn = new Trajectory();
                    // we have to try catch this everytime because otherwise we would have to tell the compiler that this method
                    // throws the IOException which is linux specific
                    try {
                        toReturn = TrajectoryUtil.fromPathweaverJson(path);
                    } catch (IOException ex) {
                        DriverStation.reportError("Failed to load trajectory for: " + path.getFileName(), true);
                    }
                    return toReturn;
                };

        // lambda to build Ramsete Command
        ramseteCommandFactory = (Trajectory trajectory) -> new RamseteCommand(
                        trajectory,
                        chassis::getPose,
                        new RamseteController(2, 0.7),
                        chassis.getFeedforward(),
                        chassis.getmKinematics(),
                        chassis::getSpeeds,
                        chassis.getleftPIDController(),
                        chassis.getRightPIDController(),
                        chassis::setOutput,
                        chassis
                );

        // lambda to build Auton commands
        autonCmdFactory = (Trajectory trajectory) -> {
            Pose2d posStart, posEnd;
            try {
                posStart = trajectory.getInitialPose();
                posEnd = trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters;
            }
            catch (IndexOutOfBoundsException ex) {
                posStart = new Pose2d(0, 0, new Rotation2d(0));
                posEnd = posStart;
            }
            return new AutonCommand(ramseteCommandFactory.apply(trajectory), posStart, posEnd);
        };

    }

    public SequentialCommandGroup add3Ball() {
        AutonCommand PathOne = autonCmdFactory.apply(trajectoryFactory.apply(Filesystem.getDeployDirectory().toPath().resolve("paths/3Ball/Start.wpilib.json")));
        CommandBase deployIntake = new DeployAndSpintake(container.getIntake(), container.getMagazine(), 1);
        RamseteCommand GoToFirstBall = ramseteCommandFactory.apply(trajectoryFactory.apply(Filesystem.getDeployDirectory().toPath().resolve("paths/3Ball/FirstBall3Ball.wpilib.json")));
        RamseteCommand goToFirstShoot = ramseteCommandFactory.apply(trajectoryFactory.apply(Filesystem.getDeployDirectory().toPath().resolve("paths/3Ball/FirstShoot.wpilib.json")));
        ParallelCommandGroup shoot = new ParallelCommandGroup(new FaceTarget(container.getChassis(), container.getLimelight()), new SetFlywheelRPM(container.getShooter(), container.getMagazine(), container.getLimelight()));
        RamseteCommand toSecondBall = ramseteCommandFactory.apply(trajectoryFactory.apply(Filesystem.getDeployDirectory().toPath().resolve("paths/3Ball/ToSecondBall.wpilib.json")));
        RamseteCommand pickupSecondBall = ramseteCommandFactory.apply(trajectoryFactory.apply(Filesystem.getDeployDirectory().toPath().resolve("paths/3Ball/GoThroughSecond.wpilib.json")));
        CommandBase deployIntake2 = new DeployAndSpintake(container.getIntake(), container.getMagazine(), 1);
        RamseteCommand SecondBallAndShoot = ramseteCommandFactory.apply(trajectoryFactory.apply(Filesystem.getDeployDirectory().toPath().resolve("paths/3Ball/SecondBallAndShoot.wpilib.json")));
        CommandBase shoot2 = new SetFlywheelRPM(container.getShooter(), container.getMagazine(), container.getLimelight());

        SequentialCommandGroup commandGroup =
                new SequentialCommandGroup(
                        PathOne.getCmd(),
                        new ParallelDeadlineGroup(GoToFirstBall, deployIntake),
                        goToFirstShoot,
                        shoot,
                        toSecondBall,
                        new ParallelDeadlineGroup(pickupSecondBall, deployIntake2),
                        SecondBallAndShoot,
                        shoot2
                );

        m_autonChooser.addOption("3Ball", new AutonCommand(commandGroup, PathOne.getStartPosition()));

        return commandGroup;
    }

    public void AddThreeBallPoseTwo() {
        AutonCommand PathOne = autonCmdFactory.apply(trajectoryFactory.apply(Filesystem.getDeployDirectory().toPath().resolve("paths/3Ball2/GoToFirstBall.wpilib.json")));
        CommandBase deployIntake = new DeployAndSpintake(container.getIntake(), container.getMagazine(), 1);
        AutonCommand DriveToShoot = autonCmdFactory.apply(trajectoryFactory.apply(Filesystem.getDeployDirectory().toPath().resolve("paths/3Ball2/DriveToShoot.wpilib.json")));
        RamseteCommand spin = ramseteCommandFactory.apply(TrajectoryGenerator.generateTrajectory(List.of(PathOne.getEndPosition(), DriveToShoot.getStartPosition()), config));
        ParallelCommandGroup shoot = new ParallelCommandGroup(new FaceTarget(container.getChassis(), container.getLimelight()), new SetFlywheelRPM(container.getShooter(), container.getMagazine(), container.getLimelight()));

        SequentialCommandGroup commandGroup =
                new SequentialCommandGroup(
                        new ParallelDeadlineGroup(PathOne.getCmd(),deployIntake),
                        spin,
                        DriveToShoot.getCmd(),
                        shoot
                );
        m_autonChooser.addOption("3Ball2", new AutonCommand(commandGroup, PathOne.getStartPosition()));
    }

    public void Add1Ball() {
        AutonCommand Move = autonCmdFactory.apply(trajectoryFactory.apply(Filesystem.getDeployDirectory().toPath().resolve("paths/Move/MoveOut.wpilib.json")));
        ParallelCommandGroup shoot = new ParallelCommandGroup(new FaceTarget(container.getChassis(), container.getLimelight()), new SetFlywheelRPM(container.getShooter(), container.getMagazine(), container.getLimelight()));

        SequentialCommandGroup commandGroup =
                new SequentialCommandGroup(
                        Move.getCmd(),
                        shoot
                );
        m_autonChooser.addOption("1Ball", new AutonCommand(commandGroup, Move.getStartPosition()));
    }

    public void add5Ball() {
        AutonCommand ToFirstBall = autonCmdFactory.apply(trajectoryFactory.apply(Filesystem.getDeployDirectory().toPath().resolve("path/5Ball/ToFirstBall5Ball.wpilib.json")));
        CommandBase deployIntake = new DeployAndSpintake(container.getIntake(), container.getMagazine(), 1);
        ParallelCommandGroup shoot = new ParallelCommandGroup(new FaceTarget(container.getChassis(), container.getLimelight()), new SetFlywheelRPM(container.getShooter(), container.getMagazine(), container.getLimelight()));
        CommandBase deployIntake2 = new DeployAndSpintake(container.getIntake(), container.getMagazine(), 1);
        AutonCommand ToSecondBall = autonCmdFactory.apply(trajectoryFactory.apply(Filesystem.getDeployDirectory().toPath().resolve("path/5Ball/ToSecondBall5Ball.wpilib.json")));
        ParallelCommandGroup shoot2 = new ParallelCommandGroup(new FaceTarget(container.getChassis(), container.getLimelight()), new SetFlywheelRPM(container.getShooter(), container.getMagazine(), container.getLimelight()));
        AutonCommand ToThirdBall = autonCmdFactory.apply(trajectoryFactory.apply(Filesystem.getDeployDirectory().toPath().resolve("path/5Ball/ToThridBall5Ball.wpilib.json")));
        CommandBase deployIntake3 = new DeployAndSpintake(container.getIntake(), container.getMagazine(), 1);
        AutonCommand PickUpThirdBall = autonCmdFactory.apply(trajectoryFactory.apply(Filesystem.getDeployDirectory().toPath().resolve("path/5Ball/PickUpThirdBal5Ball..wpilib.json")));
        AutonCommand ReverseToShoot = autonCmdFactory.apply(trajectoryFactory.apply(Filesystem.getDeployDirectory().toPath().resolve("path/5Ball/reverseToShoot5Ball.wpilib.json")));
        AutonCommand ToShoot = autonCmdFactory.apply(trajectoryFactory.apply(Filesystem.getDeployDirectory().toPath().resolve("path/5Ball/ToShoot5Ball.wpilib.json")));
        ParallelCommandGroup shoot3 = new ParallelCommandGroup(new FaceTarget(container.getChassis(), container.getLimelight()), new SetFlywheelRPM(container.getShooter(), container.getMagazine(), container.getLimelight()));

        SequentialCommandGroup commandGroup =
                new SequentialCommandGroup(
                        new ParallelDeadlineGroup(ToFirstBall.getCmd(),deployIntake),
                        shoot,
                        new ParallelDeadlineGroup(ToSecondBall.getCmd(),deployIntake2),
                        shoot2,
                        ToThirdBall.getCmd(),
                        new ParallelDeadlineGroup(PickUpThirdBall.getCmd(),deployIntake3),
                        ReverseToShoot.getCmd(),
                        ToShoot.getCmd(),
                        shoot3
                );
        m_autonChooser.addOption("5Ball", new AutonCommand(commandGroup, ToFirstBall.getStartPosition()));
    }
/*
    public SequentialCommandGroup addTestRoutine(){
        AutonCommand PathOne = autonCmdFactory.apply(trajectoryFactory.apply(Filesystem.getDeployDirectory().toPath().resolve("paths/Forward1M.wpilib.json")));
        RamseteCommand PathTwo = testPath;
        CommandBase deployIntake = new DeployAndSpintake(container.getIntake(), container.getMagazine(), 1);

            SequentialCommandGroup group = new SequentialCommandGroup(
                    new ParallelDeadlineGroup(PathOne.getCmd(), deployIntake),
                    PathTwo
            );

            m_autonChooser.addOption("Routine Test", new AutonCommand(group, PathOne.getPosition()));
            return group;
    }*/

    /**
     * Add all the commands to auton chooser
     * Load all the trajectories on init
     * Adds every command without being wrapped in anything
     * DO NOT USE FOR PATHS MORE COMPLEX THAN MOTORS GO BRRR
     */
    public void addAllCommands() {
        File dir = Filesystem.getDeployDirectory();

        ArrayList<File> files = new ArrayList<>(List.of(dir.listFiles()));

        for (int i = 0; i < files.size(); i++) {
            File currentFile = files.get(i);
            String currentFileName = currentFile.getName();

            // BFS check
            if (currentFile.isDirectory()) {
                // Should act as a BFS
                files.addAll(List.of(currentFile.listFiles()));
                continue;
            }

            // continue if it is not a path weaver file
            if (!(currentFileName.contains("wpilib"))) {
                continue;
            }

            AutonCommand command = autonCmdFactory.apply(trajectoryFactory.apply(Path.of(currentFile.getAbsolutePath())));

            // set the default to speedTrain.wpilib.json
            if (currentFileName.contains("speedTrain")) {
                m_autonChooser.setDefaultOption(currentFileName.substring(0, currentFileName.indexOf('.')), command);
            }

            // chooser options
            m_autonChooser.addOption(currentFileName.substring(0, currentFileName.indexOf('.')), command);
        }
        SmartDashboard.putData(m_autonChooser);
    }


    /**
     * this should be an S curve
     */
    public void generateTestPath() {
        Chassis chassis = container.getChassis();
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(List.of(
                chassis.getPose(),
                    new Pose2d(0.8, 0, new Rotation2d(0))
                // new Pose2d(3, 0, new Rotation2d(0))
        ), config);
        testPath = autonCmdFactory.apply(trajectory);
        m_autonChooser.addOption("Test Path", testPath);
    }

    public AutonCommand getTestPath() {
        return testPath;
    }

    public AutonCommand getCommand() {
        return m_autonChooser.getSelected();
    }

    public Function<Trajectory, RamseteCommand> getRamseteCommandFactory() {
        return ramseteCommandFactory;
    }

    public TrajectoryConfig getConfig() {
        return config;
    }
}
