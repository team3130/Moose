package frc.robot.SupportingClassess;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
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
import frc.robot.commands.Shooter.Shoot;
import frc.robot.subsystems.Chassis;

import java.io.File;
import java.io.IOException;
import java.nio.file.FileStore;
import java.nio.file.Files;
import java.nio.file.Path;
import java.sql.Driver;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

public class Chooser {
    private SendableChooser<String> m_autonChooser;
    private RobotContainer container;
    private HashMap<String, RamseteCommand> paths;

    public Chooser(SendableChooser<String> m_autonChooser, RobotContainer container) {
        this.m_autonChooser = m_autonChooser;
        this.container = container;

        TrajectoryConfig config = new TrajectoryConfig(1.3, 0.5);

        config.setKinematics(container.getChassis().getmKinematics());
    }

    public SequentialCommandGroup add3Ball() {
        Chassis chassis = container.getChassis();
        // lambda to build trajectories
        Function<Path, Trajectory> trajectoryFactory =
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
        Function<Trajectory, RamseteCommand> cmdFactory = (Trajectory trajectory) -> new RamseteCommand(
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

        File dir = new File("home/lvuser/deploy/paths/3Ball");

        // needs to be tuned
        double firstBallPickupTime = 2;

        RamseteCommand PathOne = cmdFactory.apply(trajectoryFactory.apply(Filesystem.getDeployDirectory().toPath().resolve("Start.wpilib.json")));
        Command deployIntake = new TimedDeployAndSpintake(container.getIntake(), container.getMagazine(), firstBallPickupTime);
        RamseteCommand GoToFirstBall = cmdFactory.apply(trajectoryFactory.apply(Filesystem.getDeployDirectory().toPath().resolve("FirstBall.wpilib.json")));
        RamseteCommand goToFirstShoot = cmdFactory.apply(trajectoryFactory.apply(Filesystem.getDeployDirectory().toPath().resolve("Start.wpilib.json")));
        ParallelCommandGroup shoot = new ParallelCommandGroup(new FaceTarget(container.getChassis()), new Shoot(container.getShooter()));
        RamseteCommand toSecondBall = cmdFactory.apply(trajectoryFactory.apply(Filesystem.getDeployDirectory().toPath().resolve("ToSecondBall.wpilib.json")));
        // deploy again in parallel
        RamseteCommand SecondBallAndShoot = cmdFactory.apply(trajectoryFactory.apply(Filesystem.getDeployDirectory().toPath().resolve("SecondBallAndShoot.wpilib.json")));
        // shoot again
        return new SequentialCommandGroup(PathOne, new ParallelCommandGroup(deployIntake, GoToFirstBall), goToFirstShoot, shoot, toSecondBall, new ParallelCommandGroup(deployIntake, SecondBallAndShoot), shoot);
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
                files.addAll(List.of(files.get(i).listFiles()));
                continue;
            }

            Trajectory trajectory = new Trajectory();

            try {
                Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(files.get(i).getName());
                trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            }

            catch (IOException ex) {
                DriverStation.reportError("Unable to open trajectory: " + files.get(i).getName(), ex.getStackTrace());
            }

            // creating a Ramsete command which is used in AutonInit
            RamseteCommand command = new RamseteCommand(
                    trajectory,
                    chassis::getPose,
                    new RamseteController(2.0, 0.7),
                    chassis.getFeedforward(),
                    chassis.getmKinematics(),
                    chassis::getSpeeds,
                    chassis.getleftPIDController(),
                    chassis.getRightPIDController(),
                    chassis::setOutput,
                    chassis
            );

            command.addRequirements(chassis);
            command.setName(files.get(i).getName());

            // add the name to the map
            paths.put(files.get(i).getName(), command);

            // chooser options
            m_autonChooser.addOption(files.get(i).getName(), files.get(i).getName());
        }
        SmartDashboard.putData(m_autonChooser);
    }

    public RamseteCommand getCommand() {
        return paths.get(m_autonChooser.getSelected());
    }

}
