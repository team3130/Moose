package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Command_GO_BRRRRRRRRRRRRRRRRRRRRRRRR;
import frc.robot.commands.FlyWheel_Command;
import frc.robot.subsystems.WHEEL_GO_BRRRRRRRRRRRRRR;
import frc.robot.subsystems.FlyWheel;

/**
 * All objects that are going to be used that are instantiated once should be defined and accessible here
 */
public class RobotContainer {
    // define subsystems here
    private WHEEL_GO_BRRRRRRRRRRRRRR m_BRRR = new WHEEL_GO_BRRRRRRRRRRRRRR();
    private FlyWheel m_flywheel = new FlyWheel();
    // reminder that Singletons are deprecated, please do not use them even for subsystems
    // EX: private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

    // make getters for subsystems here
    public WHEEL_GO_BRRRRRRRRRRRRRR getM_BRRR() {return m_BRRR;}
    public FlyWheel getM_flywheel() {return m_flywheel;}

    public RobotContainer() {
        defineButtonBindings();
    }

    // Joysticks
    public static Joystick m_driverGamepad = new Joystick(0);
    public static Joystick m_weaponsGamepad = new Joystick(1);

    private void defineButtonBindings() {
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_X).whenHeld(new Command_GO_BRRRRRRRRRRRRRRRRRRRRRRRR(m_BRRR));
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_A).whenHeld(new FlyWheel_Command(m_flywheel));

    }
}
