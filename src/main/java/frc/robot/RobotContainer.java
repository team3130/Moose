package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.CommandEX;
import frc.robot.commands.Command_GO_BRRRRRRRRRRRRRRRRRRRRRRRR;
import frc.robot.subsystems.MotorEX;
import frc.robot.subsystems.WHEEL_GO_BRRRRRRRRRRRRRR;

/**
 * All objects that are going to be used that are instantiated once should be defined and accessible here
 */
public class RobotContainer {
    // define subsystems here
    private MotorEX mEX = new MotorEX();
    // reminder that Singletons are deprecated, please do not use them even for subsystems
    // EX: private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

    // make getters for subsystems here
    public MotorEX getM_BRRR() {return mEX;}

    public RobotContainer() {
        defineButtonBindings();
    }

    // Joysticks
    public static Joystick m_driverGamepad = new Joystick(0);
    public static Joystick m_weaponsGamepad = new Joystick(1);

    private void defineButtonBindings() {
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_X).whenHeld(new CommandEX(mEX));

    }
}
