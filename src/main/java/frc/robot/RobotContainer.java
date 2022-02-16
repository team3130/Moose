package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Intake.Motor1;
import frc.robot.subsystems.Intake;

/**
 * All objects that are going to be used that are instantiated once should be defined and accessible here
 */
public class RobotContainer {
    // define subsystems here
    Intake m_intake = new Intake();
    // reminder that Singletons are deprecated, please do not use them even for subsystems
    // EX: private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

    // make getters for subsystems here
    public Intake m_intake() {return m_intake;}

    public RobotContainer() {
        defineButtonBindings();
    }

    // Joysticks
    public static Joystick m_driverGamepad = new Joystick(0);
    public static Joystick m_weaponsGamepad = new Joystick(1);

    private void defineButtonBindings() {
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_X).whenHeld(new Motor1(m_intake, 0.75));
    }
}
