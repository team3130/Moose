// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.SupportingClassess.Chooser;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  CommandScheduler m_scheduler = CommandScheduler.getInstance();

  private final SendableChooser<String> m_autonChooser = new SendableChooser<>();
  private String m_autoSelected;

  private final SendableChooser<String> m_chooser_driver = new SendableChooser<>();
  private final SendableChooser<String> m_chooser_weapons = new SendableChooser<>();

  private RobotContainer m_robotContainer;
  private Chooser m_chooser;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    SmartDashboard.putData("Auton", m_autonChooser);
    // driver options
    m_chooser_driver.setDefaultOption("Cody", "Cody");
    m_chooser_driver.addOption("Maddie", "Maddie");
    SmartDashboard.putData("Driver", m_chooser_driver);
    // weapon options
    m_chooser_weapons.setDefaultOption("Ben", "Ben");
    m_chooser_weapons.addOption("Parker", "Parker");
    SmartDashboard.putData("Weapons", m_chooser_weapons);
    m_robotContainer = new RobotContainer();
    m_chooser = new Chooser(m_autonChooser, m_robotContainer);
    m_chooser.add3Ball();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    m_robotContainer.outputToShuffleBoard();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_scheduler.schedule(m_chooser.getCommand());
    // week 0 auton attempt
    /*
    m_scheduler.schedule(
            new SequentialCommandGroup(
                    new AutonDrive(m_robotContainer.getChassis(), 0.75),
                    new AutonShoot(m_robotContainer.getShooter()),
                    new AutonDrive(m_robotContainer.getChassis(), 0.6)
            )
    );
    */
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    m_scheduler.run();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    m_robotContainer.defineButtonBindings(m_chooser_driver, m_chooser_weapons);
    m_robotContainer.teleopInit();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    m_scheduler.run();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    m_scheduler.clearButtons();
    m_robotContainer.disable();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    m_robotContainer.defineButtonBindings(m_chooser_driver, m_chooser_weapons);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when test mode is enabled */
  public void simulationInit() {
    m_robotContainer.defineButtonBindings(m_chooser_driver, m_chooser_weapons);
  }

  /** This Function is called periodically in simulations such as glass */
  @Override
  public void simulationPeriodic() {}
}
