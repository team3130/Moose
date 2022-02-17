package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Chassis.SimpleDrive;
import frc.robot.commands.Climber.Climber_Motor;
import frc.robot.commands.Hopper.Hopper_Motor;
import frc.robot.commands.Intake.Intake_Motor;
import frc.robot.commands.Intake.Pnumeatics;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;

/**
 * All objects that are going to be used that are instantiated once should be defined and accessible here
 */
public class RobotContainer {
    // define subsystems here
    Intake m_intake = new Intake();
    Chassis m_chassis = new Chassis();
    Climber m_climber = new Climber();
    Hopper m_hopper = new Hopper();

    // reminder that Singletons are deprecated, please do not use them even for subsystems
    // EX: private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

    // make getters for subsystems here
    public Intake m_intake() {
        return m_intake;
    }

    public Chassis m_chassis() {
        return m_chassis;
    }

    public Climber m_climber() {
        return m_climber;
    }

    public Hopper m_hopper() {
        return m_hopper;
    }

    public RobotContainer() {
        defineButtonBindings();
    }

    // Joysticks
    public static Joystick m_driverGamepad = new Joystick(0);
    public static Joystick m_weaponsGamepad = new Joystick(1);

    private void defineButtonBindings() {
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_X).whenHeld(new Intake_Motor(m_intake, 0.75));
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_B).whenHeld(new Intake_Motor(m_intake, -0.75));
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_A).whenHeld(new Pnumeatics(m_intake, true));
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_B).whenHeld(new Pnumeatics(m_intake, false));
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_LJOYSTICKPRESS).whenHeld(new SimpleDrive(m_chassis));
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_LBUMPER).whenHeld(new Climber_Motor(m_climber, 0.1));
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_RBUMPER).whenHeld(new Climber_Motor(m_climber, -0.1));
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_MENU).whenHeld(new Hopper_Motor(m_hopper, 0.2));
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_WINDOW).whenHeld(new Hopper_Motor(m_hopper, -0.2));
    }
}
