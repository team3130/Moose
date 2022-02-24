package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.deployintake;
import frc.robot.commands.spintake;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.intakesubsystem;

/**
 * All objects that are going to be used that are instantiated once should be defined and accessible here
 */
public class RobotContainer {
    // define subsystems here
    Chassis m_chassis = new Chassis();
    intakesubsystem m_intakesubsystem = new intakesubsystem();


    // reminder that Singletons are deprecated, please do not use them even for subsystems
    // EX: private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();


    // make getters for subsystems here
    public intakesubsystem getintakesubsystem() {
        return m_intakesubsystem;
    }

    public RobotContainer() {
        defineButtonBindings();
        m_chassis.setDefaultCommand(new DefaultDrive(m_chassis));
    }

    // Joysticks
    public static Joystick m_driverGamepad = new Joystick(0);
    public static Joystick m_weaponsGamepad = new Joystick(1);

    private void defineButtonBindings() {
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_A).whenPressed(new deployintake(m_intakesubsystem));
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_X).whenHeld(new spintake(m_intakesubsystem));
    }


    public void outputToShuffleBoard() {
        m_chassis.outputToShuffleboard();
    }

}
