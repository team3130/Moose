package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

/**
 * All objects that are going to be used that are instantiated once should be defined and accessible here
 */
public class RobotContainer {
    // define subsystems here
    private grapplingHook m_grapplingHook = new grapplingHook
    // reminder that Singletons are deprecated, please do not use them even for subsystems
    // EX: private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();


    // make getters for subsystems here


    public grapplingHook getM_grapplingHook() {
        reurn m_grapplingHook;
    }

    public RobotContainer() {
        defineButtonBindings();
    }

    // Joysticks
    public static Joystick m_driverGamepad = new Joystick(0);
    public static Joystick m_weaponsGamepad = new Joystick(1);

    private void defineButtonBindings() {
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTM_X).whenHeld(new gatekeepGrapplingHook());

    }
}
