package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Chassis.Shift;
import frc.robot.commands.Chassis.faceTarget;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.subsystems.MotorIntake;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.Chassis.DefaultDrive;
import frc.robot.commands.Intake.deployintake;
import frc.robot.commands.Intake.spintake;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Intake;

/**
 * All objects that are going to be used that are instantiated once should be defined and accessible here
 */
public class RobotContainer {
    // define subsystems here
    Shooter m_shooter = new Shooter();
    Chassis m_chassis = new Chassis();
    MotorIntake m_motorintake = new MotorIntake();


    // reminder that Singletons are deprecated, please do not use them even for subsystems
    // EX: private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

    public MotorIntake getIntake() {
        return m_motorintake;
    }

    public Chassis getChassis() {
        return m_chassis;
    }

    public Shooter getShooter() {
        return m_shooter;
    }

    public RobotContainer() {
        defineButtonBindings();
        m_chassis.setDefaultCommand(new DefaultDrive(m_chassis));
    }

    // Joysticks
    public static Joystick m_driverGamepad = new Joystick(0);
    public static Joystick m_weaponsGamepad = new Joystick(1);

    private void defineButtonBindings() {
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_A).whenHeld(new Shoot(m_shooter));
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_Y).whenPressed(new deployintake(m_intake));
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_X).whenHeld(new spintake(m_motorintake));
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_B).whenPressed(new faceTarget(m_chassis));
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_LJOYSTICKPRESS).whenPressed(new Shift(m_chassis));
    }

    public void outputToShuffleBoard() {
        m_chassis.outputToShuffleboard();
    }

}
