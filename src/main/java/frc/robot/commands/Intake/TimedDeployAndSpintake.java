package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;

public class TimedDeployAndSpintake extends CommandBase {
    // defining an instance to be used throughout the command and to be instantiated in the constructor of type parameter
    private final Intake m_intake;
    private final Magazine m_magazine;
    private final int direction;
    private final double time;
    private final Timer timer;

    /**
     * Meant to be run in a sequential command group with {@link TimedSpintake}
     * @param intake {@link Intake}
     */
    public TimedDeployAndSpintake(Intake intake, Magazine magazine, double time) {
        //mapping to object passed through parameter
        m_intake = intake;
        m_magazine = magazine;
        m_requirements.add(m_intake);
        m_requirements.add(m_magazine);
        this.time = time;
        timer = new Timer();

        direction = 1;
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        m_intake.deployIntake(true);
        m_intake.setSpeed(0.8 * direction);
        timer.reset();
        timer.start();
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        if (timer.hasElapsed(time - RobotMap.kIntakeRetractTime)) {
            m_intake.deployIntake(false);
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
    public boolean isFinished() {return timer.get() >= time;}

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
        m_intake.deployIntake(false);
        m_intake.setSpeed(0);
    }
}