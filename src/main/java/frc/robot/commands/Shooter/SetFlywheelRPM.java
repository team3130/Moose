package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.vision.Limelight;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;

public class SetFlywheelRPM extends CommandBase {
    // defining an instance to be used throughout the command and to be instantiated in the constructor of type parameter
    private final Shooter m_shooter;
    private final Magazine m_magazine;
    private Limelight m_limelight;
    private final double timeLimit = 2;
    private final Timer timer = new Timer();

    public SetFlywheelRPM(Shooter subsystem, Magazine magazine, Limelight m_limelight) {
        //mapping to object passed through parameter
        m_shooter = subsystem;
        m_magazine = magazine;
        m_requirements.add(subsystem);
        m_requirements.add(magazine);
        this.m_limelight = m_limelight;
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        m_shooter.setHoodWheelTopSpeed(m_shooter.getHoodWheelSpeedFromShuffleboard());
        m_shooter.setFlywheelSetSpeed(m_shooter.getSpeedFromShuffleboard());
        m_shooter.feedFlywheel();
        m_shooter.feedHoodWheel();
        m_limelight.setLedState(true);
        timer.reset();
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        if (m_shooter.getRPM() >= m_shooter.getSpeedFromShuffleboard() - 50) {
            m_shooter.setIndexerPercent(0.3);
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
        if (m_magazine.isEmpty()) {
            timer.start();
        }
        // we do it again for the second ball
        return timer.get() >= timeLimit && m_magazine.isEmpty();
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
        m_shooter.setFlywheelSpeed(0);
        m_shooter.setHoodWheelTopSpeed(0);
        m_shooter.setIndexerPercent(0);
        m_limelight.setLedState(false);
        timer.stop();
    }
}
