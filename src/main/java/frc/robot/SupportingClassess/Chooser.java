package frc.robot.SupportingClassess;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.commands.Chassis.FaceTarget;
import frc.robot.commands.Intake.DeployAndSpintake;
import frc.robot.commands.Intake.TimedDeployAndSpintake;
import frc.robot.commands.Shooter.SetFlywheelRPM;
import frc.robot.subsystems.Chassis;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Function;

public class Chooser implements Runnable{
    private SendableChooser<String> m_autonChooser;
    private RobotContainer container;
    private HashMap<String, CommandBase> paths;
    private Function<Path, Trajectory> trajectoryFactory;
    private Function<Trajectory, RamseteCommand> cmdFactory;
    private TrajectoryConfig config;

    private RamseteCommand testPath;

    public Chooser(SendableChooser<String> m_autonChooser, RobotContainer container) {
        this.m_autonChooser = m_autonChooser;
        this.container = container;

        Chassis chassis = container.getChassis();

        DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                        chassis.getFeedforward(),
                        chassis.getmKinematics(),
                        12);

        config = new TrajectoryConfig(RobotMap.kMaxVelocityMPS, RobotMap.kMaxAccelerationMPS);
        config.setKinematics(container.getChassis().getmKinematics()).addConstraint(autoVoltageConstraint);

        paths = new HashMap<>();

        // lambda to build trajectories
        trajectoryFactory =
                (Path path) -> {
                    Trajectory toReturn = new Trajectory();
                    // we have to try catch this everytime because otherwise we would have to tell the compiler that this method
                    // throws the IOException which is linux specific
                    try {
                        toReturn = TrajectoryUtil.fromPathweaverJson(path);
                    } catch (IOException ex) {
                        DriverStation.reportError("Failed to load trajectory", true);
                    }
                    return toReturn;
                };

        // lambda to build RamseteCommands
        cmdFactory = (Trajectory trajectory) -> new RamseteCommand(
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
    }

    public SequentialCommandGroup add3Ball() {
        // needs to be tuned
        double firstBallPickupTime = 2;

        RamseteCommand PathOne = cmdFactory.apply(trajectoryFactory.apply(Filesystem.getDeployDirectory().toPath().resolve("paths/3Ball/Start.wpilib.json")));
        CommandBase deployIntake = new DeployAndSpintake(container.getIntake(), container.getMagazine(), 1);
        RamseteCommand GoToFirstBall = cmdFactory.apply(trajectoryFactory.apply(Filesystem.getDeployDirectory().toPath().resolve("paths/3Ball/FirstBall.wpilib.json")));
        RamseteCommand goToFirstShoot = cmdFactory.apply(trajectoryFactory.apply(Filesystem.getDeployDirectory().toPath().resolve("paths/3Ball/Start.wpilib.json")));
        ParallelCommandGroup shoot = new ParallelCommandGroup(new FaceTarget(container.getChassis(), container.getLimelight()), new SetFlywheelRPM(container.getShooter(), container.getLimelight()));
        RamseteCommand toSecondBall = cmdFactory.apply(trajectoryFactory.apply(Filesystem.getDeployDirectory().toPath().resolve("paths/3Ball/ToSecondBall.wpilib.json")));
        RamseteCommand pickupSecondBall = cmdFactory.apply(trajectoryFactory.apply(Filesystem.getDeployDirectory().toPath().resolve("paths/3Ball/GoThroughSecond.wpilib.json")));
        CommandBase deployIntake2 = new DeployAndSpintake(container.getIntake(), container.getMagazine(), 1);
        RamseteCommand SecondBallAndShoot = cmdFactory.apply(trajectoryFactory.apply(Filesystem.getDeployDirectory().toPath().resolve("paths/3Ball/SecondBallAndShoot.wpilib.json")));
        ParallelCommandGroup shoot2 = new ParallelCommandGroup(new FaceTarget(container.getChassis(), container.getLimelight()), new SetFlywheelRPM(container.getShooter(), container.getLimelight()));

        SequentialCommandGroup commandGroup =
                new SequentialCommandGroup(
                        PathOne,
                        new ParallelDeadlineGroup(GoToFirstBall, deployIntake),
                        goToFirstShoot,
                        shoot,
                        toSecondBall,
                        new ParallelDeadlineGroup(pickupSecondBall, deployIntake2),
                        SecondBallAndShoot,
                        shoot2
                );

        paths.put("3Ball", commandGroup);
        
        m_autonChooser.addOption("3Ball", "3Ball");
        m_autonChooser.setDefaultOption("3Ball", "3Ball");

        return commandGroup;
    }

    /**
     * Add all the commands to auton chooser
     * Load all the trajectories on init
     * Adds every command without being wrapped in anything
     * DO NOT USE FOR PATHS MORE COMPLEX THAN MOTORS GO BRRR
     */
    public void addAllCommands() {
        File dir = new File("/home/lvuser/deploy/paths/");

        Chassis chassis = container.getChassis();

        ArrayList<File> files = new ArrayList<>(List.of(dir.listFiles()));

        for (int i = 0; i < files.size(); i++) {
            if (files.get(i).isDirectory()) {
                // should act as a BFS
                files.addAll(List.of(files.get(i).listFiles()));
                continue;
            }

            if (files.get(i).getName().lastIndexOf('.') >= 0) {
                if (!files.get(i).getName().substring(files.get(i).getName().lastIndexOf('.')).equals("json")) {
                    continue;
                }
            }

            RamseteCommand command = cmdFactory.apply(trajectoryFactory.apply(Path.of(files.get(i).getAbsolutePath())));

            command.addRequirements(chassis);
            command.setName(files.get(i).getName());

            // add the name to the map
            paths.put(files.get(i).getName(), command);

            // chooser options
            m_autonChooser.addOption(files.get(i).getName().substring(0, files.get(i).getName().indexOf('.')), files.get(i).getName());
        }
        m_autonChooser.setDefaultOption("CPath", "CPath.wpilib.json");
        SmartDashboard.putData(m_autonChooser);
    }

    /**
     * this should be an S curve
     */
    public void generateTestPath() {
        Chassis chassis = container.getChassis();
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(List.of(
                chassis.getPose(),
                new Pose2d(3, 1, new Rotation2d(0))
        ), config);
        testPath = cmdFactory.apply(trajectory);
    }

    public RamseteCommand getTestPath() {
        return testPath;
    }

    public CommandBase getCommand() {
        return paths.get(m_autonChooser.getSelected());
    }

    /**
     * When an object implementing interface {@code Runnable} is used
     * to create a thread, starting the thread causes the object's
     * {@code run} method to be called in that separately executing
     * thread.
     * <p>
     * The general contract of the method {@code run} is that it may
     * take any action whatsoever.
     *
     * @see Thread#run()
     */
    @Override
    public void run() {
        addAllCommands();
    }
}
