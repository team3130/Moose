package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.SupportingClassess.Chooser;
import frc.robot.SupportingClassess.GeneralUtils;
import frc.robot.commands.Chassis.FaceTarget;
import frc.robot.commands.Chassis.Shift;
import frc.robot.commands.Chassis.resetOdometery;
import frc.robot.commands.Climber.spinClimberWinches;
import frc.robot.commands.Climber.ToggleClimber;
import frc.robot.commands.Hood.SpinHood;
import frc.robot.commands.Hood.ZeroHood;
import frc.robot.commands.Intake.DeployAndSpintake;
import frc.robot.commands.Intake.TimedSpintake;
import frc.robot.commands.Magazine.Spinzine;
import frc.robot.commands.Shooter.SetFlywheelRPM;
import frc.robot.commands.Shooter.SpinHoodWheel;
import frc.robot.commands.Shooter.SpindexTimed;
import frc.robot.controls.TriggerButton;
import frc.robot.sensors.vision.Limelight;
import frc.robot.sensors.vision.WheelSpeedCalculations;
import frc.robot.subsystems.*;
import frc.robot.commands.Chassis.DefaultDrive;
import frc.robot.commands.Intake.DeployIntake;
import frc.robot.commands.Intake.Spintake;

import java.util.ArrayList;
import java.util.List;

/**
 * All objects that are going to be used that are instantiated once should be defined and accessible here
 */
public class RobotContainer {
    private ArrayList<GeneralUtils> m_generalUtils;
    // Supporting classes
    protected WheelSpeedCalculations m_wheelSpeedCalculations = new WheelSpeedCalculations();
    protected Limelight m_limelight = new Limelight();

    // define subsystems here
    Shooter m_shooter = new Shooter();
    Chassis m_chassis = new Chassis();
    Intake m_intake = new Intake();
    Magazine m_magazine = new Magazine();
    Hood m_hood = new Hood();
    Climber m_climber = new Climber();

    protected Chooser m_chooser;

    // reminder that Singletons are deprecated, please do not use them even for subsystems
    // EX: private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

    public Intake getIntake() {
        return m_intake;
    }

    public Chassis getChassis() {
        return m_chassis;
    }

    public Shooter getShooter() {
        return m_shooter;
    }

    public Limelight getLimelight() {
        return m_limelight;
    }

    public Climber getClimber() {
        return m_climber;
    }

    public Magazine getMagazine() {return m_magazine;}

    public Hood getHood() {
        return m_hood;
    }

    public RobotContainer() {
        m_generalUtils = new ArrayList<>();
        m_generalUtils.addAll(List.of(m_chassis, m_shooter, m_intake, m_magazine, m_limelight, m_hood));
        m_chassis.setDefaultCommand(new DefaultDrive(m_chassis));
    }

    public Chooser getChooser() {
        return m_chooser;
    }

    // Joysticks
    public static Joystick m_driverGamepad = new Joystick(0);
    public static Joystick m_weaponsGamepad = new Joystick(1);

    public void defineButtonBindings(SendableChooser<String> m_chooser_driver, SendableChooser<String> m_chooser_weapons) {
        // driver controls
        if (m_chooser_driver.getSelected().equals("Cody")) {
            new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_B).whenHeld(new FaceTarget(m_chassis, m_limelight));
            new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_LJOYSTICKPRESS).whenPressed(new Shift(m_chassis));
            new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_RBUMPER).whenPressed(new DeployAndSpintake(m_intake, m_magazine, 1)).whenReleased(new TimedSpintake(m_intake, m_magazine));
        }
        else if (m_chooser_driver.getSelected().equals("Maddie")) {
            new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_A).whenHeld(new FaceTarget(m_chassis, m_limelight));
            new TriggerButton(m_driverGamepad, RobotMap.LST_AXS_RTRIGGER).whenPressed(new DeployAndSpintake(m_intake, m_magazine, 1)).whenReleased(new TimedSpintake(m_intake, m_magazine));
            new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_LJOYSTICKPRESS).whenPressed(new Shift(m_chassis));
        }
        else if (m_chooser_driver.getSelected().equals("Test")) {
            new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_RJOYSTICKPRESS).whenHeld(new FaceTarget(m_chassis, m_limelight));
            new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_RBUMPER).whenHeld(new SpinHood(m_hood, 1));
            new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_LBUMPER).whenHeld(new SpinHood(m_hood, -1));
            new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_LJOYSTICKPRESS).whenPressed(new Shift(m_chassis));
            new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_MENU).whenPressed(new DeployIntake(m_intake));
            new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_X).whenHeld(new ZeroHood(m_hood));
            new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_Y).whenHeld(new SpindexTimed(m_shooter, 0.1));
            new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_A).whenHeld(new Spinzine(m_magazine, 1));
            new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_B).whenPressed(new DeployAndSpintake(m_intake, m_magazine, 1)).whenReleased(new TimedSpintake(m_intake, m_magazine));
            new TriggerButton(m_driverGamepad, RobotMap.LST_AXS_LTRIGGER).whenHeld(new SetFlywheelRPM(m_shooter, m_magazine, m_limelight));
            new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_WINDOW).whenHeld(new SpinHoodWheel(m_shooter));
            new TriggerButton(m_driverGamepad, RobotMap.LST_AXS_RTRIGGER).whenHeld(new SetFlywheelRPM(m_shooter, m_magazine, m_limelight));
        }

        // weapons controls
        if (m_chooser_weapons.getSelected().equals("Ben")) {
            new TriggerButton(m_weaponsGamepad, RobotMap.LST_AXS_RTRIGGER).whenPressed(new DeployAndSpintake(m_intake, m_magazine, 1)).whenReleased(new TimedSpintake(m_intake, m_magazine));
            new TriggerButton(m_weaponsGamepad, RobotMap.LST_AXS_LTRIGGER).whenHeld(new DeployAndSpintake(m_intake, m_magazine, -1));
            new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_MENU).whenPressed(new DeployIntake(m_intake));
            new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_A).whenPressed(new ToggleClimber(m_climber));
            new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_B).whenHeld(new SetFlywheelRPM(m_shooter, m_magazine, m_limelight));
            new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_X).whenHeld(new Spinzine(m_magazine, 1));
            new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_Y).whenHeld(new Spinzine(m_magazine, -1));
            new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_RBUMPER).whenHeld(new spinClimberWinches(m_climber, -1));
            new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_LBUMPER).whenHeld(new spinClimberWinches(m_climber, 1));
        }
        else if (m_chooser_weapons.getSelected().equals("Parker")) {
            new TriggerButton(m_weaponsGamepad, RobotMap.LST_AXS_LTRIGGER).whenHeld(new Spinzine(m_magazine, 1)); // ltrigger
            new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_RBUMPER).whenPressed(new DeployAndSpintake(m_intake, m_magazine, 1)).whenReleased(new TimedSpintake(m_intake, m_magazine)); //rbumber
            new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_LBUMPER).whenHeld(new SetFlywheelRPM(m_shooter, m_magazine, m_limelight));
            new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_Y).whenHeld(new DeployAndSpintake(m_intake, m_magazine, -1)); // y
            new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_X).whenPressed(new DeployIntake(m_intake)); //x
            new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_B).whenHeld(new Spintake(m_intake, -1)); //b
            new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_A).whenHeld(new Spintake(m_intake, 1)); //a
        }
    }

    public void outputToShuffleBoard() {
        m_generalUtils.forEach(GeneralUtils::outputToShuffleboard);
    }

    public void teleopInit() {
        m_chassis.setDefaultCommand(new DefaultDrive(m_chassis));
        m_generalUtils.forEach(GeneralUtils::teleopInit);
    }

    public void disable() {
        m_generalUtils.forEach(GeneralUtils::disable);
    }
}
