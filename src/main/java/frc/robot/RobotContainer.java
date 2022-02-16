package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SpinClimberWinch;
import frc.robot.subsystems.Climber;

/**
 * All objects that are going to be used that are instantiated once should be defined and accessible here
 */
public class RobotContainer {
    // define subsystems here
    private Climber m_climber = new Climber();

    // reminder that Singletons are deprecated, please do not use them even for subsystems
    // EX: private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();


    // make getters for subsystems here


    public RobotContainer() {
        defineButtonBindings();
    }

    // Joysticks
    public static Joystick m_driverGamepad = new Joystick(0);
    public static Joystick m_weaponsGamepad = new Joystick(1);

    private void defineButtonBindings() {
        // reverse
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_A).whenHeld(new SpinClimberWinch(m_climber, -1));
        // forward
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_B).whenHeld(new SpinClimberWinch(m_climber, 1));

    }

}
