package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Motor_Subsystem;
import frc.robot.subsystems.Motor_Subsystem2;
 
/**
 * All objects that are going to be used that are instantiated once should be defined and accessible here
 */
public class RobotContainer {
    // define subsystems here
    private Motor_Subsystem m_Motor_Subsystem = new Motor_Subsystem();
    private Motor_Subsystem2 m_Motor_Subsystem2 = new Motor_Subsystem2();
    public static boolean boost = false;
    public static boolean right = false;
    public static boolean left = false;
    // reminder that Singletons are deprecated, please do not use them even for subsystems
    // EX: private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

    // make getters for subsystems here
    public Motor_Subsystem getM_Motor_Subsystem() {
        return m_Motor_Subsystem;
    }

    public Motor_Subsystem2 getM_Motor_Subsystem2() {
        return m_Motor_Subsystem2;
    }

    public RobotContainer() {
        defineButtonBindings();
    }

    // Joysticks
    public static Joystick m_driverGamepad = new Joystick(0);
    public static Joystick m_weaponsGamepad = new Joystick(1);

    private void defineButtonBindings() {
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_LBUMPER).whenHeld(new Drive(m_Motor_Subsystem, m_Motor_Subsystem2, 0.3, 0));
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_RBUMPER).whenHeld(new Drive(m_Motor_Subsystem, m_Motor_Subsystem2, 0, 0.3));
    }
}
