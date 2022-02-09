package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Boost;
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
    boolean boost = false;
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
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_B).whenHeld(new Boost(boost));
        new JoystickButton(m_driverGamepad, RobotMap.LST_POV_N).whenHeld(new Drive(m_Motor_Subsystem, m_Motor_Subsystem2, 0.1, 0.1, boost));
        new JoystickButton(m_driverGamepad, RobotMap.LST_POV_NE).whenHeld(new Drive(m_Motor_Subsystem, m_Motor_Subsystem2, 0.1, 0.075, boost));
        new JoystickButton(m_driverGamepad, RobotMap.LST_POV_E).whenHeld(new Drive(m_Motor_Subsystem, m_Motor_Subsystem2, 0.075, -0.075, boost));
        new JoystickButton(m_driverGamepad, RobotMap.LST_POV_SE).whenHeld(new Drive(m_Motor_Subsystem, m_Motor_Subsystem2, -0.1, -0.075, boost));
        new JoystickButton(m_driverGamepad, RobotMap.LST_POV_S).whenHeld(new Drive(m_Motor_Subsystem, m_Motor_Subsystem2, -0.075, -0.075, boost));
        new JoystickButton(m_driverGamepad, RobotMap.LST_POV_SW).whenHeld(new Drive(m_Motor_Subsystem, m_Motor_Subsystem2, -0.075, 0.1, boost));
        new JoystickButton(m_driverGamepad, RobotMap.LST_POV_W).whenHeld(new Drive(m_Motor_Subsystem, m_Motor_Subsystem2, -0.075, 0.075, boost));
        new JoystickButton(m_driverGamepad, RobotMap.LST_POV_NW).whenHeld(new Drive(m_Motor_Subsystem, m_Motor_Subsystem2, 0.075, 0.1, boost));
    }

}
