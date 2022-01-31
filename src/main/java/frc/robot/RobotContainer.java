package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.toggleClimber;
import frc.robot.subsystems.Climber;

/**
 * All objects that are going to be used that are instantiated once should be defined and accessible here
 */
public class RobotContainer {
    // define subsystems here
    Climber m_climber = new Climber();
    // reminder that Singletons are deprecated, please do not use them even for subsystems
    // EX: private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();


    // make getters for subsystems here

    public Climber getM_climber() {
        return m_climber;
    }

    public RobotContainer() {
        defineButtonBindings();
    }

    // Joysticks
    public static Joystick m_driverGamepad = new Joystick(0);
    public static Joystick m_weaponsGamepad = new Joystick(1);

    private void defineButtonBindings(m_driverGamepad, RobotMap.LST_BTN_B).whenPressed(new toggleClimber(m_hood)); {}

}
