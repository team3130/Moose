package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Chassis.Shift;
import frc.robot.commands.Chassis.FaceTarget;
import frc.robot.commands.Intake.DeployAndSpintake;
import frc.robot.commands.Intake.TimedSpintake;
import frc.robot.commands.Magazine.Spinzine;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.controls.TriggerButton;
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
    private ArrayList<SubsystemBased> subsystems;
    // define subsystems here
    Shooter m_shooter = new Shooter();
    Chassis m_chassis = new Chassis();
    Intake m_intake = new Intake();
    Magazine m_magazine = new Magazine();

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

    public Magazine getMagazine() {return m_magazine;}

    public RobotContainer() {
        subsystems = new ArrayList<>();
        subsystems.addAll(List.of(m_chassis, m_shooter, m_intake, m_magazine));
        m_chassis.setDefaultCommand(new DefaultDrive(m_chassis));
    }

    // Joysticks
    public static Joystick m_driverGamepad = new Joystick(0);
    public static Joystick m_weaponsGamepad = new Joystick(1);

    public void defineButtonBindings(SendableChooser<String> m_chooser_driver, SendableChooser<String> m_chooser_weapons) {
        // driver controls
        if (m_chooser_driver.getSelected() == null) {
            System.out.println("m_chooser_driver.getSelected() = null!");
        } else if (m_chooser_driver.getSelected().equals("Cody")) {
            new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_B).whenPressed(new FaceTarget(m_chassis));
            new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_LJOYSTICKPRESS).whenPressed(new Shift(m_chassis));
            new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_RBUMPER).whenPressed(new DeployAndSpintake(m_intake, m_magazine, 1)).whenReleased(new TimedSpintake(m_intake, m_magazine));
        } else if (m_chooser_driver.getSelected().equals("Maddie")) {
            new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_A).whenPressed(new FaceTarget(m_chassis));
            new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_RBUMPER).whenPressed(new DeployAndSpintake(m_intake, m_magazine, 1)).whenReleased(new TimedSpintake(m_intake, m_magazine));
            new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_LJOYSTICKPRESS).whenPressed(new Shift(m_chassis));
        }        

        // weapons controls
        if (m_chooser_weapons.getSelected() == null) {
            System.out.println("m_chooser_weapons.getSelected() = null!");
        } else if (m_chooser_weapons.getSelected().equals("Ben")) {
            new TriggerButton(m_weaponsGamepad, RobotMap.LST_AXS_RTRIGGER).whenPressed(new DeployAndSpintake(m_intake, m_magazine, 1)).whenReleased(new TimedSpintake(m_intake, m_magazine));
            new TriggerButton(m_weaponsGamepad, RobotMap.LST_AXS_LTRIGGER).whenHeld(new DeployAndSpintake(m_intake, m_magazine, -1));
            new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_LBUMPER).whenHeld(new Shoot(m_shooter));
            new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_MENU).whenPressed(new DeployIntake(m_intake));
            new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_A).whenHeld(new Spinzine(m_magazine, 1));
            new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_B).whenHeld(new Spinzine(m_magazine, -1));
            new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_X).whenHeld(new Spintake(m_intake, -1));
            new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_Y).whenHeld(new Spintake(m_intake, 1));
        } else if (m_chooser_weapons.getSelected().equals("Parker")) {
            new TriggerButton(m_weaponsGamepad, RobotMap.LST_AXS_RTRIGGER).whenPressed(new DeployAndSpintake(m_intake, m_magazine, 1)).whenReleased(new TimedSpintake(m_intake, m_magazine)); //rbumber
            new TriggerButton(m_weaponsGamepad, RobotMap.LST_AXS_LTRIGGER).whenHeld(new Spinzine(m_magazine, 1)); // ltrigger
            new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_LBUMPER).whenHeld(new Shoot(m_shooter));
            new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_Y).whenHeld(new DeployAndSpintake(m_intake, m_magazine, -1)); // y
            new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_X).whenPressed(new DeployIntake(m_intake)); //x
            new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_B).whenHeld(new Spintake(m_intake, -1)); //b
            new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_A).whenHeld(new Spintake(m_intake, 1)); //a
        }
    }

    public void outputToShuffleBoard() {
        subsystems.forEach(SubsystemBased::outputToShuffleboard);
    }

    public void teleopInit() {
        subsystems.forEach(SubsystemBased::teleopInit);
    }

    public void disable() {
        subsystems.forEach(SubsystemBased::disable);
    }
}
