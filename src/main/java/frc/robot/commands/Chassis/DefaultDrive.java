package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.subsystems.Chassis;

public class DefaultDrive extends CommandBase {
    // defining an instance to be used throughout the command and to be instantiated in the constructor of type parameter
    private final Chassis m_chassis; //TODO: rename this to the subsystem this is assigned to

    public DefaultDrive(Chassis chassis) {
        //mapping to object passed through parameter
        m_chassis = chassis;
        m_requirements.add(chassis);
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        m_chassis.configRampRate(RobotMap.kMaxRampRate);
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        double moveSpeed = -RobotContainer.m_driverGamepad.getRawAxis(1); //joystick's y axis is inverted
        if (m_chassis.isShifted()) {
            moveSpeed *= RobotMap.kMaxHighGearDriveSpeed * (m_chassis.getMoveSpeedSensitivityFromShuffleboard() / 10);
        }
        double turnSpeed = RobotContainer.m_driverGamepad.getRawAxis(4) * RobotMap.kMaxHighGearDriveSpeed * (m_chassis.getTurnSpeedSensitivityFromShuffleboard() / 10);

        m_chassis.driveArcade(moveSpeed, turnSpeed * RobotMap.kMaxTurnThrottle, true);
    }

    /**
     * <p>
     * Returns whether this command has finished. Once a command finishes -- indicated by
     * this method returning true -- the scheduler will call its {@link #end(boolean)} method.
     * </p><p>
     * Returning false will result in the command never ending automatically. It may still be
     * cancelled manually or interrupted by another command. Hard coding this command to always
     * return true will result in the command executing once and finishing immediately. It is
     * recommended to use * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand}
     * for such an operation.
     * </p>
     *
     * @return whether this command has finished.
     */
    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    /**
     * The action to take when the command ends. Called when either the command
     * finishes normally -- that is it is called when {@link #isFinished()} returns
     * true -- or when  it is interrupted/canceled. This is where you may want to
     * wrap up loose ends, like shutting off a motor that was being used in the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        m_chassis.configRampRate(0);
    }
}
