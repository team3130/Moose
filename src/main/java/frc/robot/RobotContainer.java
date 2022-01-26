package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.Command_GO_BRRR;
import frc.robot.subsystems.WheelBrrr;


/**
 * All objects that are going to be used that are instantiated once should be defined and accessible here
 */
public class RobotContainer {
    // define subsystems here
    private WheelBrrr m_wheelBRRR = new WheelBrrr();
    // reminder that Singletons are deprecated, please do not use them even for subsystems
    // EX: private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

    // make getters for subsystems here
    public WheelBrrr getM_WHEELBRRR() {return m_wheelBRRR;}

    public RobotContainer() {
        defineButtonBindings();
    }

    // Joysticks
    public static Joystick m_driverGamepad = new Joystick(0);
    public static Joystick m_weaponsGamepad = new Joystick(1);

    private void defineButtonBindings() {
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_Y).whenHeld(new Command_GO_BRRR(m_wheelBRRR));
    }
}
