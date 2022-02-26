package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Shooter;

import java.util.Set;

public class AutonShoot extends CommandBase {
    // defining an instance to be used throughout the command and to be instantiated in the constructor of type parameter
    private final Shooter m_shooter;
    private final double timeLimit = 5;
    private final Timer timer = new Timer();

    public AutonShoot(Shooter subsystem) {
        //mapping to object passed through parameter
        m_shooter = subsystem;
        m_requirements.add(subsystem);
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        double shooterSpeed = 3000; //TODO: find correct speed
        double indexerSpeed = 2000;
        
        m_shooter.setSpeed(shooterSpeed); 

        if (m_shooter.getRPM() >= shooterSpeed - 50) {
            m_shooter.setIndexerSpeed(indexerSpeed);//POTENTIAL FAILURE POINT: if shooter doesn't work tomorrow it could be this rpm implementation
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
        return timer.get() >= timeLimit;
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
        m_shooter.setIndexerSpeed(0);
        m_shooter.setSpeed(0);
        timer.stop();
    }
}

