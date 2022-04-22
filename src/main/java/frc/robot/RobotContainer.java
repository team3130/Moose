package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.SupportingClassess.AutonCommand;
import frc.robot.SupportingClassess.Chooser;
import frc.robot.SupportingClassess.GeneralUtils;
import frc.robot.commands.Chassis.*;
import frc.robot.commands.Climber.ToggleClimber;
import frc.robot.commands.Climber.spinClimberWinches;
import frc.robot.commands.Intake.DeployAndSpintake;
import frc.robot.commands.Intake.DeployAndSpintakeMagazineBack;
import frc.robot.commands.Intake.DeployIntake;
import frc.robot.commands.Intake.TimedSpintake;
import frc.robot.commands.Magazine.Spinzine;
import frc.robot.commands.Shooter.BenShoot;
import frc.robot.commands.Shooter.ChooseFlywheelRPM;
import frc.robot.commands.Shooter.SetFlywheelRPM;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.commands.ZeroClimber;
import frc.robot.controls.TriggerButton;
import frc.robot.sensors.vision.Limelight;
import frc.robot.sensors.vision.WheelSpeedCalculations;
import frc.robot.subsystems.*;

import java.util.ArrayList;
import java.util.List;

/**
 * All objects that are going to be used that are instantiated once should be defined and accessible here
 */
public class RobotContainer {
    private ArrayList<GeneralUtils> m_generalUtils;
    // Supporting classes
    protected WheelSpeedCalculations m_wheelSpeedCalculations = new WheelSpeedCalculations(WheelSpeedCalculations.CurveMechanism.SHOOTER);
    protected Limelight m_limelight = new Limelight();

    // define subsystems here
    Shooter m_shooter = new Shooter(m_limelight, m_wheelSpeedCalculations);
    Chassis m_chassis = new Chassis();
    Intake m_intake = new Intake();
    Magazine m_magazine = new Magazine();
    Climber m_climber = new Climber();
    ChassisCooler m_chassiscooler = new ChassisCooler();
    protected Chooser m_chooser;

    // reminder that Singletons are deprecated, please do not use them even for subsystems
    // EX: private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

    public Intake getIntake() {
        return m_intake;
    }

    public Chassis getChassis() {
        return m_chassis;
    }

    public Shooter getShooter() {
        return m_shooter;
    }

    public Limelight getLimelight() {
        return m_limelight;
    }

    public Climber getClimber() {
        return m_climber;
    }

    public Magazine getMagazine() {
        return m_magazine;
    }

    public ChassisCooler getChassisCooler() {
        return m_chassiscooler;
    }

    public Chooser getChooser() {
        return m_chooser;
    }

    public RobotContainer(SendableChooser<AutonCommand> autonChooser) {
        m_generalUtils = new ArrayList<>();
        m_generalUtils.addAll(List.of(m_chassis, m_shooter, m_intake, m_magazine, m_limelight, m_wheelSpeedCalculations));
        m_chassis.setDefaultCommand(new DefaultDrive(m_chassis));
        m_climber.setDefaultCommand(new spinClimberWinches(m_climber));
        m_chooser = new Chooser(autonChooser, this);

        m_climber.setDefaultCommand(new spinClimberWinches(m_climber));
        // m_chassiscooler.setDefaultCommand(new CoolerCommand(m_chassiscooler, m_chassis));
    }

    // Joysticks
    public static Joystick m_driverGamepad = new Joystick(0);
    public static Joystick m_weaponsGamepad = new Joystick(1);

    public void defineButtonBindings(SendableChooser<String> m_chooser_driver) {
        // driver controls
        if (m_chooser_driver.getSelected().equals("Cody")) {
            new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_B).whenHeld(new FaceTarget(m_chassis, m_limelight));
            new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_LJOYSTICKPRESS).whenPressed(new Shift(m_chassis));

        }
        else if (m_chooser_driver.getSelected().equals("Test")) {
            new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_B).whenHeld(new SpinChassisToAngle(m_chassis, 45));
            new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_LJOYSTICKPRESS).whenPressed(new Shift(m_chassis));
            new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_MENU).whenPressed(new DeployIntake(m_intake));
            new TriggerButton(m_driverGamepad, RobotMap.LST_AXS_LTRIGGER).whenHeld(new DeployAndSpintake(m_intake, m_magazine, 1));
            new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_A).whenHeld(new ChooseFlywheelRPM(m_shooter, m_magazine, 850));
            new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_RBUMPER).whenHeld(new Shoot(m_shooter, m_magazine, m_chassis, m_limelight));
            new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_LBUMPER).whenHeld(new SetFlywheelRPM(m_shooter, m_magazine, m_limelight));
            new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_Y).whenHeld(new DeployAndSpintakeMagazineBack(m_intake, m_magazine, 1));
            // new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_Y).whenHeld(new SpindexTimed(m_shooter, 0.1));
            // new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_A).whenHeld(new Spinzine(m_magazine, 1));
            new TriggerButton(m_driverGamepad, RobotMap.LST_AXS_RTRIGGER).whenPressed(new DeployAndSpintake(m_intake, m_magazine, 1)).whenReleased(new TimedSpintake(m_intake, m_magazine));
            new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_WINDOW).whenPressed(() -> m_limelight.toggleLEDstate());
            new JoystickButton(m_driverGamepad, RobotMap. LST_BTN_X).whenHeld(new CoolerCommand(m_chassiscooler, m_chassis));
        }

        // weapons controls
        new TriggerButton(m_weaponsGamepad, RobotMap.LST_AXS_RTRIGGER).whenPressed(new DeployAndSpintake(m_intake, m_magazine, 1)).whenReleased(new TimedSpintake(m_intake, m_magazine));
        new TriggerButton(m_weaponsGamepad, RobotMap.LST_AXS_LTRIGGER).whenHeld(new DeployAndSpintake(m_intake, m_magazine, -1));
        new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_RBUMPER).whenHeld(new BenShoot(m_shooter, m_chassis, m_limelight));
        new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_LBUMPER).whenHeld(new ChooseFlywheelRPM(m_shooter, m_magazine, 1000));
//            new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_RBUMPER).whenHeld(new ChooseFlywheelRPM(m_shooter, m_magazine, 3300));
//            new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_LBUMPER).whenHeld(new ChooseFlywheelRPM(m_shooter, m_magazine, 3500));
        new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_B).whenHeld(new SetFlywheelRPM(m_shooter, m_magazine, m_limelight));
        new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_MENU).whenPressed(new DeployIntake(m_intake));
        new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_A).whenPressed(new ToggleClimber(m_climber));
        new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_X).whenHeld(new Spinzine(m_magazine, 1));
        new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_Y).whenHeld(new DeployAndSpintakeMagazineBack(m_intake, m_magazine, 1));
        new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_WINDOW).whenPressed(() -> m_limelight.toggleLEDstate());
//            new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_B).whenHeld(new SetFlywheelRPM(m_shooter, m_magazine, m_limelight));
        new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_LJOYSTICKPRESS).whenPressed(() -> m_wheelSpeedCalculations.ModifySlider(false));
        new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_RJOYSTICKPRESS).whenPressed(() -> m_wheelSpeedCalculations.ModifySlider(true));
        new TriggerButton(m_weaponsGamepad, RobotMap.LST_POV_S).whenHeld(new ZeroClimber(getClimber()));
    }

    public void outputToShuffleBoard() {
        m_generalUtils.forEach(GeneralUtils::outputToShuffleboard);
    }

    public void teleopInit() {
        m_generalUtils.forEach(GeneralUtils::teleopInit);
    }

    public void disable() {
        m_generalUtils.forEach(GeneralUtils::disable);
    }
}
