package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.commose;
import frc.robot.subsystems.MotorThing;

/**
 * All objects that are going to be used that are instantiated once should be defined and accessible here
 */
public class RobotContainer {
    // define subsystems here
    private MotorThing m_MotorThing = new MotorThing();
    // reminder that Singletons are deprecated, please do not use them even for subsystems
    // EX: private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();


    // make getters for subsystems here
    private MotorThing getM_MotorThing() {return m_MotorThing;}

    public RobotContainer() {
        defineButtonBindings();
    }

    // Joysticks
    public static Joystick m_driverGamepad = new Joystick(0);
    public static Joystick m_weaponsGamepad = new Joystick(1);

    private void defineButtonBindings() {
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_X).whenHeld(new commose(m_MotorThing));}

}
