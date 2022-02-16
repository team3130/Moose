package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Climb;
import frc.robot.subsystems.Climber;

/**
 * All objects that are going to be used that are instantiated once should be
 * defined and accessible here
 */
public class RobotContainer {
    // define subsystems here
    private Climber m_climber = new Climber();

    // reminder that Singletons are deprecated, please do not use them even for
    // subsystems
    // EX: private final ExampleSubsystem m_exampleSubsystem = new
    // ExampleSubsystem();

    // make getters for subsystems here
    public Climber getM_Climber() {
        return m_climber;
    }

    public RobotContainer() {
        defineButtonBindings();
    }

    // Joysticks
    public static Joystick m_driverGamepad = new Joystick(0);
    public static Joystick m_weaponsGamepad = new Joystick(1);

    private void defineButtonBindings() {
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_X).whenHeld(new Climb(m_climber, 1));
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_Y).whenHeld(new Climb(m_climber, -1));

    }

}