package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Chassis.Shift;
import frc.robot.commands.Shoot;
import frc.robot.commands.Spindexer;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.magazziCommand;
import frc.robot.subsystems.Magazine;
import frc.robot.commands.Chassis.DefaultDrive;
import frc.robot.commands.Intake.deployintake;
import frc.robot.commands.Intake.spintake;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.intakesubsystem;

/**
 * All objects that are going to be used that are instantiated once should be defined and accessible here
 */
public class RobotContainer {
    // define subsystems here
    Shooter m_shooter = new Shooter();
    Indexer m_indexer = new Indexer();
//    Magazine m_magazine = new Magazine();
    Chassis m_chassis = new Chassis();
    intakesubsystem m_intakesubsystem = new intakesubsystem();


    // reminder that Singletons are deprecated, please do not use them even for subsystems
    // EX: private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();


    // make getters for subsystems here
/*    public Magazine getMagazine() {
        return m_magazine;
    }*/
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
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_A).whenHeld(new Shoot(m_shooter, 1));
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_B).whenHeld(new Shoot(m_shooter, -1));
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_Y).whenHeld(new Spindexer(m_indexer, 1));
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_X).whenHeld(new Spindexer(m_indexer, -1));
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_LBUMPER).whenHeld(new deployintake(m_intakesubsystem));
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_RBUMPER).whenHeld(new spintake(m_intakesubsystem));
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_LJOYSTICKPRESS).whenHeld(new Shift(m_chassis));/\
    }


    public void outputToShuffleBoard() {
        m_chassis.outputToShuffleboard();
    }

}
