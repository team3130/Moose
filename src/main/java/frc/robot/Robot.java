// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.SupportingClassess.AutonCommand;
import frc.robot.SupportingClassess.Chooser;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  CommandScheduler m_scheduler = CommandScheduler.getInstance();

  private final SendableChooser<AutonCommand> m_autonChooser = new SendableChooser<>();

  private final SendableChooser<String> m_chooser_driver = new SendableChooser<>();

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
    m_chooser_driver.addOption("Test", "Test");
    m_chooser_driver.addOption("Kid", "Kid");
    m_chooser_driver.addOption("Maddie", "Maddie");
    m_chooser_driver.setDefaultOption("Cody", "Cody");
    SmartDashboard.putData("Driver", m_chooser_driver);

    m_robotContainer = new RobotContainer(m_autonChooser, m_chooser_driver);
    m_chooser = m_robotContainer.getChooser();
//    m_chooser.addAllCommands();
//    m_chooser.add3Ball();
    m_chooser.addThreeBallPathTwo();
    m_chooser.addTwoBall();
    m_chooser.addFiller();
    m_chooser.add3Ball3();
    m_chooser.add2Ball2();
//    m_chooser.generateTestPath();
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
    AutonCommand cmd = m_autonChooser.getSelected();
    m_robotContainer.getLimelight().setLedState(true);

    m_robotContainer.getChassis().resetOdometry(cmd.getStartPosition());
    m_scheduler.schedule(cmd.getCmd());
    m_robotContainer.getLimelight().setLedState(true);


   // week 0 auton attempt
/*    m_scheduler.schedule(
            new SequentialCommandGroup(
                    new BadAutonDrive(m_robotContainer.getChassis()),
                    new BadAutonShoot(m_robotContainer.getShooter(), m_robotContainer.getMagazine())
                    
            )
    );*/


  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    m_scheduler.run();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    m_robotContainer.defineButtonBindings();
    m_robotContainer.teleopInit();
    m_scheduler.cancelAll();
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
    m_scheduler.cancelAll();
    m_robotContainer.getChassis().configRampRate(0);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    m_robotContainer.defineButtonBindings();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when test mode is enabled */
  public void simulationInit() {
    m_robotContainer.defineButtonBindings();
  }

  /** This Function is called periodically in simulations such as glass */
  @Override
  public void simulationPeriodic() {}
}
