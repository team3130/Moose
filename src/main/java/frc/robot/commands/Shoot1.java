package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;

public class Shoot1 extends CommandBase {
    // defining an instance to be used throughout the command and to be instantiated in the constructor of type parameter
    private final Shooter m_shooter;
    private final Indexer m_indexer;
    private final Magazine m_magazine;
    private int sign;

    public Shoot1(Shooter shooter, Indexer indexer, Magazine magazine, int sign) {
        //mapping to object passed through parameter
        m_shooter = shooter;
        m_indexer = indexer;
        m_magazine = magazine;

        m_requirements.add(shooter);
        m_requirements.add(indexer);
        m_requirements.add(magazine);

        this.sign = sign;
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        m_shooter.setSpeed(m_shooter.getSpeedFromShuffleboard() * sign);
        m_magazine.spinny(0.2 * sign);
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        if(m_shooter.getRPM() > 0.8 * m_shooter.getSpeedFromShuffleboard()) {
            m_indexer.setSpeed(m_indexer.getPercentFromShuffleboard() * sign);
        }
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
        m_shooter.spinMotor(0);
    }
}
