/*Copyright (c) FIRST and other WPILib contributors. Open Source Software; you can modify and/or share it under the terms ofthe WPILib BSD license file in the root directory of this project.*/package frc.robot;import edu.wpi.first.wpilibj.TimedRobot;import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;import edu.wpi.first.wpilibj2.command.CommandScheduler;import edu.wpi.first.wpilibj2.command.ScheduleCommand;public class Robot extends TimedRobot {private String m_autoSelected;private final SendableChooser<String> m_chooser = new SendableChooser<>();RobotContainer m_robotContainer;CommandScheduler m_scheduler = CommandScheduler.getInstance();CommandScheduler scheduler = CommandScheduler.getInstance();@Override public void robotInit() {m_chooser.setDefaultOption("Default Auto", RobotMap.kDefaultAuto);m_chooser.addOption("My Auto", RobotMap.kCustomAuto);SmartDashboard.putData("Auto choices", m_chooser);m_robotContainer = new RobotContainer();}@Override public void robotPeriodic() {m_robotContainer.outputToShuffleBoard();}@Override public void autonomousInit() {m_autoSelected = m_chooser.getSelected();System.out.println("Auto selected: " + m_autoSelected);}@Override public void autonomousPeriodic() {switch (m_autoSelected) {case RobotMap.kCustomAuto:break;case RobotMap.kDefaultAuto:default:break;}}@Override public void teleopInit() {}@Override public void teleopPeriodic() {m_scheduler.run();}@Override public void disabledInit() {}@Override public void disabledPeriodic() {}@Override public void testInit() {}@Override public void testPeriodic() {}public void simulationInit() {}@Override public void simulationPeriodic() {}}