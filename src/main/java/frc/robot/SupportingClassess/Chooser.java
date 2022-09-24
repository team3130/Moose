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
import frc.robot.commands.Chassis.SpinChassisToAbsoluteAngle;
import frc.robot.commands.Chassis.SpinChassisToAngle;
import frc.robot.commands.Intake.DeployAndSpintake;
import frc.robot.commands.Shooter.SetFlywheelRPM;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.commands.doNothing;
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
                10);

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

    public void add3Ball() {
        AutonCommand PathOne = autonCmdFactory.apply(trajectoryFactory.apply(Filesystem.getDeployDirectory().toPath().resolve("paths/3Ball/Start.wpilib.json")));
        CommandBase deployIntake = new DeployAndSpintake(container.getIntake(), container.getMagazine(), 1, container.getShooter());
        RamseteCommand GoToFirstBall = ramseteCommandFactory.apply(trajectoryFactory.apply(Filesystem.getDeployDirectory().toPath().resolve("paths/3Ball/FirstBall3Ball.wpilib.json")));
        RamseteCommand goToFirstShoot = ramseteCommandFactory.apply(trajectoryFactory.apply(Filesystem.getDeployDirectory().toPath().resolve("paths/3Ball/FirstShoot.wpilib.json")));
        ParallelDeadlineGroup shoot = new ParallelDeadlineGroup(new Shoot(container.getShooter(), container.getMagazine(), container.getChassis(), container.getLimelight()), new FaceTarget(container.getChassis(), container.getLimelight()));
        AutonCommand toSecondBall = autonCmdFactory.apply(trajectoryFactory.apply(Filesystem.getDeployDirectory().toPath().resolve("paths/3Ball/ToSecondBall.wpilib.json")));
        RamseteCommand pickupSecondBall = ramseteCommandFactory.apply(trajectoryFactory.apply(Filesystem.getDeployDirectory().toPath().resolve("paths/3Ball/GoThroughSecond.wpilib.json")));
        CommandBase deployIntake2 = new DeployAndSpintake(container.getIntake(), container.getMagazine(), 1, container.getShooter());
        RamseteCommand SecondBallAndShoot = ramseteCommandFactory.apply(trajectoryFactory.apply(Filesystem.getDeployDirectory().toPath().resolve("paths/3Ball/SecondBallAndShoot.wpilib.json")));
        CommandBase shoot2 =  new ParallelDeadlineGroup(new Shoot(container.getShooter(), container.getMagazine(), container.getChassis(), container.getLimelight()),  new FaceTarget(container.getChassis(), container.getLimelight()));

        CommandBase commandGroup =
            new SequentialCommandGroup(
                    new ParallelDeadlineGroup(
                    new SequentialCommandGroup(
                            PathOne.getCmd(),
                            GoToFirstBall,
                            goToFirstShoot),
                    new DeployAndSpintake(container.getIntake(), container.getMagazine(), 1, container.getShooter())),
                    shoot,
                    new ParallelDeadlineGroup(
                            new SequentialCommandGroup(
                            toSecondBall.getCmd(),
                            pickupSecondBall,
                            SecondBallAndShoot),
                            new DeployAndSpintake(container.getIntake(), container.getMagazine(), 1, container.getShooter())
                    ),
                    shoot2
        );

        m_autonChooser.addOption("3Ball", new AutonCommand(commandGroup, PathOne.getStartPosition()));
    }

    public void addThreeBallPathTwo() {
        AutonCommand pathOne = autonCmdFactory.apply(
                trajectoryFactory.apply(
                        Filesystem.getDeployDirectory().toPath().resolve("paths/3Ball2/GoToFirstBall3Ball2.wpilib.json")
                )
        );
        CommandBase shoot = new Shoot(container.getShooter(), container.getMagazine(), container.getChassis(), container.getLimelight());

        CommandBase spin1 = new SpinChassisToAngle(container.getChassis(), 180);

        AutonCommand pathTwo = autonCmdFactory.apply(
                trajectoryFactory.apply(
                        Filesystem.getDeployDirectory().toPath().resolve("paths/3Ball2/GoToSecondBall3Ball2.wpilib.json")
                )
        );

        CommandBase spin2 = new SpinChassisToAngle(container.getChassis(), 180);
        RamseteCommand pathThree = ramseteCommandFactory.apply(
                trajectoryFactory.apply(
                        Filesystem.getDeployDirectory().toPath().resolve("paths/3Ball2/GoToShoot3Ball2.wpilib.json")
                )
        );
        CommandBase shoot2 = new Shoot(container.getShooter(), container.getMagazine(), container.getChassis(), container.getLimelight());

        SequentialCommandGroup commandGroup =
                new SequentialCommandGroup(
                    new ParallelDeadlineGroup(pathOne.getCmd(), new DeployAndSpintake(container.getIntake(), container.getMagazine(), 1, container.getShooter())),
                    shoot,
                    spin1,
                    new ParallelDeadlineGroup(pathTwo.getCmd(), new DeployAndSpintake(container.getIntake(), container.getMagazine(), 1, container.getShooter())),
                    spin2,
                    pathThree,
                    shoot2
                );

        m_autonChooser.addOption("3Ball2", new AutonCommand(commandGroup, pathOne.getStartPosition()));
    }

    public void addFiller() {
        m_autonChooser.addOption("Nothing", new AutonCommand(new doNothing(container.getChassis()), new Pose2d()));
    }

    public void addTwoBall() {
        AutonCommand PathOne = autonCmdFactory.apply(
                trajectoryFactory.apply(
                        Filesystem.getDeployDirectory().toPath().resolve("paths/2Ball/GoToFirstBall.wpilib.json")
                )
        );
        CommandBase deployIntake = new DeployAndSpintake(container.getIntake(), container.getMagazine(), 1, container.getShooter());

        AutonCommand DriveToShoot = autonCmdFactory.apply(
                trajectoryFactory.apply(
                        Filesystem.getDeployDirectory().toPath().resolve("paths/2Ball/DriveToShoot2Ball.wpilib.json")
                )
        );
        CommandBase spin = new SpinChassisToAngle(container.getChassis(), 180);

        CommandBase shoot = new Shoot(container.getShooter(), container.getMagazine(), container.getChassis(), container.getLimelight());

        SequentialCommandGroup commandGroup =
                new SequentialCommandGroup(
                        new ParallelDeadlineGroup(PathOne.getCmd(), deployIntake),
                        spin,
                        DriveToShoot.getCmd(),
                        shoot
                );

        m_autonChooser.setDefaultOption("2Ball", new AutonCommand(commandGroup, PathOne.getStartPosition()));
    }

    public void Add1Ball() {
        AutonCommand Move = autonCmdFactory.apply(
                trajectoryFactory.apply(
                        Filesystem.getDeployDirectory().toPath().resolve("paths/Move/MoveOut.wpilib.json")
                )
        );

        ParallelCommandGroup shoot = new ParallelCommandGroup(
                new FaceTarget(container.getChassis(), container.getLimelight()),
                new SetFlywheelRPM(container.getShooter(), container.getMagazine(), container.getChassis(), container.getLimelight())
        );

        SequentialCommandGroup commandGroup =
                new SequentialCommandGroup(
                        Move.getCmd(),
                        shoot
                );

        m_autonChooser.addOption("1Ball", new AutonCommand(commandGroup, Move.getStartPosition()));
    }

    public void add2Ball2() {
        AutonCommand goToFirstBall = autonCmdFactory.apply(
                trajectoryFactory.apply(
                        Filesystem.getDeployDirectory().toPath().resolve("paths/3Ball3/GoToBall3Ball3.wpilib.json")
                )
        );

        CommandBase deployIntake = new DeployAndSpintake(container.getIntake(), container.getMagazine(), 1, container.getShooter());

        AutonCommand ReverseToShoot = autonCmdFactory.apply(
                trajectoryFactory.apply(
                        Filesystem.getDeployDirectory().toPath().resolve("paths/3Ball3/GoToShoot3Ball3.wpilib.json")
                )
        );

        CommandBase shoot = new Shoot(container.getShooter(), container.getMagazine(), container.getChassis(), container.getLimelight());

        SequentialCommandGroup commandGroup = new SequentialCommandGroup(
                new ParallelDeadlineGroup(goToFirstBall.getCmd(), deployIntake), ReverseToShoot.getCmd(), shoot
                );

        m_autonChooser.setDefaultOption("Funny 2 Ball", new AutonCommand(commandGroup, goToFirstBall.getStartPosition()));
    }

    public void add5Ball() {
        AutonCommand ToFirstBall = autonCmdFactory.apply(trajectoryFactory.apply(Filesystem.getDeployDirectory().toPath().resolve("path/5Ball/ToFirstBall5Ball.wpilib.json")));
        CommandBase deployIntake = new DeployAndSpintake(container.getIntake(), container.getMagazine(), 1, container.getShooter());
        CommandBase shoot = new Shoot(container.getShooter(), container.getMagazine(), container.getChassis(), container.getLimelight());
        CommandBase deployIntake2 = new DeployAndSpintake(container.getIntake(), container.getMagazine(), 1, container.getShooter());
        AutonCommand ToSecondBall = autonCmdFactory.apply(trajectoryFactory.apply(Filesystem.getDeployDirectory().toPath().resolve("path/5Ball/ToSecondBall5Ball.wpilib.json")));
        CommandBase shoot2 = new Shoot(container.getShooter(), container.getMagazine(), container.getChassis(), container.getLimelight());
        AutonCommand ToThirdBall = autonCmdFactory.apply(trajectoryFactory.apply(Filesystem.getDeployDirectory().toPath().resolve("path/5Ball/ToThridBall5Ball.wpilib.json")));
        CommandBase deployIntake3 = new DeployAndSpintake(container.getIntake(), container.getMagazine(), 1, container.getShooter());
        AutonCommand PickUpThirdBall = autonCmdFactory.apply(trajectoryFactory.apply(Filesystem.getDeployDirectory().toPath().resolve("path/5Ball/PickUpThirdBal5Ball..wpilib.json")));
        AutonCommand ReverseToShoot = autonCmdFactory.apply(trajectoryFactory.apply(Filesystem.getDeployDirectory().toPath().resolve("path/5Ball/reverseToShoot5Ball.wpilib.json")));
        AutonCommand ToShoot = autonCmdFactory.apply(trajectoryFactory.apply(Filesystem.getDeployDirectory().toPath().resolve("path/5Ball/ToShoot5Ball.wpilib.json")));
        CommandBase shoot3 = new Shoot(container.getShooter(), container.getMagazine(), container.getChassis(), container.getLimelight());

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

    public void addWeekZero(){}
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

    public void addSpitOut() {
        AutonCommand goToFirstBall = autonCmdFactory.apply(trajectoryFactory.apply(Filesystem.getDeployDirectory().toPath().resolve("paths/2BallEject/GoToFirst2BallEject.wpilib.json")));
        CommandBase deployIntake = new DeployAndSpintake(container.getIntake(), container.getMagazine(), 1, container.getShooter());
        AutonCommand GoShoot = autonCmdFactory.apply(trajectoryFactory.apply(Filesystem.getDeployDirectory().toPath().resolve("paths/2BallEject/GoShoot2BallEject.wpilib.json")));
        CommandBase shoot = new Shoot(container.getShooter(), container.getMagazine(), container.getChassis(), container.getLimelight());
        AutonCommand GoToSecondBall = autonCmdFactory.apply(trajectoryFactory.apply(Filesystem.getDeployDirectory().toPath().resolve("paths/2BallEject/GoToSecondEjectBall2BallEject.wpilib.json")));
        CommandBase deployIntake2 = new DeployAndSpintake(container.getIntake(), container.getMagazine(), 1, container.getShooter());
        CommandBase shootIntoHanger = new DeployAndSpintake(container.getIntake(), container.getMagazine(), -1, container.getShooter());

        SequentialCommandGroup commandGroup = new SequentialCommandGroup(
                new ParallelDeadlineGroup(goToFirstBall.getCmd(), deployIntake),
                GoShoot.getCmd(),
                shoot,
                new ParallelDeadlineGroup(GoToSecondBall.getCmd(), deployIntake2),
                shootIntoHanger
        );

        m_autonChooser.addOption("2 Ball Into Hanger", new AutonCommand(commandGroup, goToFirstBall.getStartPosition()));
    }


    /**
     * this should be an S curve
     */
    public void generateTestPath() {
        Chassis chassis = container.getChassis();
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(List.of(
                chassis.getPose(),
                    new Pose2d(1, 0, new Rotation2d(0))
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

    public void add3Ball3() {
        AutonCommand goToFirstBall = autonCmdFactory.apply(
                trajectoryFactory.apply(
                        Filesystem.getDeployDirectory().toPath().resolve("paths/3Ball3/GoToBall3Ball3.wpilib.json")
                )
        );

        CommandBase deployIntake = new DeployAndSpintake(container.getIntake(), container.getMagazine(), 1, container.getShooter());

        AutonCommand ReverseToShoot = autonCmdFactory.apply(
                trajectoryFactory.apply(
                        Filesystem.getDeployDirectory().toPath().resolve("paths/3Ball3/GoToShoot3Ball3.wpilib.json")
                )
        );

        CommandBase shoot = new Shoot(container.getShooter(), container.getMagazine(), container.getChassis(), container.getLimelight());

        AutonCommand GoToLastBall = autonCmdFactory.apply(
                trajectoryFactory.apply(
                        Filesystem.getDeployDirectory().toPath().resolve("paths/3Ball3/GoToLastBall3.wpilib.json")
                )
        );

        // Sequentially this must occur before GoToLastBall it must be generated after GoToLastBall
        CommandBase spin = new SpinChassisToAbsoluteAngle(container.getChassis(), GoToLastBall.getStartPosition().getRotation().getDegrees());

        CommandBase deployIntake2 = new DeployAndSpintake(container.getIntake(), container.getMagazine(), 1, container.getShooter());

        AutonCommand GoToShootLast = autonCmdFactory.apply(
                trajectoryFactory.apply(
                        Filesystem.getDeployDirectory().toPath().resolve("paths/3Ball3/GoToShootLast3Ball3.wpilib.json")
                )
        );

        CommandBase spin2 = new SpinChassisToAbsoluteAngle(container.getChassis(), GoToShootLast.getStartPosition().getRotation().getDegrees());

        CommandBase shoot2 = new Shoot(container.getShooter(), container.getMagazine(), container.getChassis(), container.getLimelight());

        SequentialCommandGroup commandGroup = new SequentialCommandGroup(
                new ParallelDeadlineGroup(goToFirstBall.getCmd(), deployIntake),
                ReverseToShoot.getCmd(),
                shoot,
                new ParallelDeadlineGroup(GoToLastBall.getCmd(), deployIntake2),
                GoToShootLast.getCmd(),
                shoot2
        );

        m_autonChooser.addOption("Desperate 3 Ball", new AutonCommand(commandGroup, goToFirstBall.getStartPosition()));
    }

    public void generateFourBall() {
        AutonCommand goToFirstBall = autonCmdFactory.apply(
                trajectoryFactory.apply(
                        Filesystem.getDeployDirectory().toPath().resolve("paths/4Ball/GoToBall3Ball3.wpilib.json")
                )
        );

        RamseteCommand reverseToShoot = ramseteCommandFactory.apply(trajectoryFactory.apply(Filesystem.getDeployDirectory().toPath().resolve("paths/4Ball/GoToShoot3Ball3.wpilib.json")));

        CommandBase deployIntake = new DeployAndSpintake(container.getIntake(), container.getMagazine(), 1, container.getShooter());

        AutonCommand goTo3rdAnd4thBall = autonCmdFactory.apply(
                trajectoryFactory.apply(
                        Filesystem.getDeployDirectory().toPath().resolve("paths/4Ball/FourBallOne.wpilib.json")
                )
        );

        CommandBase shoot = new Shoot(container.getShooter(), container.getMagazine(), container.getChassis(), container.getLimelight());

        AutonCommand ReverseToShoot2 = autonCmdFactory.apply(
                trajectoryFactory.apply(
                        Filesystem.getDeployDirectory().toPath().resolve("paths/4Ball/ReverseAfter4thBall.wpilib.json")
                )
        );


        CommandBase deployIntake2 = new DeployAndSpintake(container.getIntake(), container.getMagazine(), 1, container.getShooter());

        AutonCommand GoToShootLast = autonCmdFactory.apply(
                trajectoryFactory.apply(
                        Filesystem.getDeployDirectory().toPath().resolve("paths/4Ball/GoToShoot4.wpilib.json")
                )
        );

        CommandBase shoot2 = new Shoot(container.getShooter(), container.getMagazine(), container.getChassis(), container.getLimelight());

        SequentialCommandGroup commandGroup = new SequentialCommandGroup(
                new ParallelDeadlineGroup(goToFirstBall.getCmd(), deployIntake),
                reverseToShoot,
                shoot,
                new ParallelDeadlineGroup(goTo3rdAnd4thBall.getCmd(), deployIntake2),
                ReverseToShoot2.getCmd(),
                GoToShootLast.getCmd(),
                shoot2
        );

        m_autonChooser.addOption("Scuffed 4 Ball", new AutonCommand(commandGroup, goToFirstBall.getStartPosition()));
    }

    public void generate3BallL() {
        AutonCommand goToFirstBall = autonCmdFactory.apply(
                trajectoryFactory.apply(
                        Filesystem.getDeployDirectory().toPath().resolve("paths/3BallL/3BallLGoToBall.wpilib.json")
                )
        );
        AutonCommand goToSecondBall = autonCmdFactory.apply(
                trajectoryFactory.apply(
                        Filesystem.getDeployDirectory().toPath().resolve("paths/3BallL/GoToShoot3BallL.wpilib.json")
                )
        );
        CommandBase deployIntake = new DeployAndSpintake(container.getIntake(), container.getMagazine(), 1, container.getShooter());
        CommandBase deployIntake2 = new DeployAndSpintake(container.getIntake(), container.getMagazine(), 1, container.getShooter());

        SpinChassisToAngle spinToShoot = new SpinChassisToAngle(container.getChassis(), 180);

        Shoot shoot = new Shoot(container.getShooter(), container.getMagazine(), container.getChassis(), container.getLimelight());

        SpinChassisToAbsoluteAngle spinToAngle = new SpinChassisToAbsoluteAngle(container.getChassis(), goToSecondBall.getStartPosition().getRotation().getDegrees());

        SpinChassisToAngle spinChassisToShoot2 = new SpinChassisToAngle(container.getChassis(), -90);

        Shoot shoot2 = new Shoot(container.getShooter(), container.getMagazine(), container.getChassis(), container.getLimelight());

        SequentialCommandGroup commandGroup = new SequentialCommandGroup(
                new ParallelDeadlineGroup(goToFirstBall.getCmd(), deployIntake), spinToShoot, shoot, spinToAngle, new ParallelDeadlineGroup(goToSecondBall.getCmd(), deployIntake2), spinChassisToShoot2, shoot2
        );

        m_autonChooser.addOption("3Ball L", new AutonCommand(commandGroup, goToFirstBall.getStartPosition()));
    }
}
