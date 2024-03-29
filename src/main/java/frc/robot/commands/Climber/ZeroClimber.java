package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ZeroClimber extends CommandBase {
    // defining an instance to be used throughout the command and to be instantiated in the constructor of type parameter
    private final Climber m_Climber;
    private boolean BrokeRight = false;
    private boolean BrokeLeft = false;

    private double RightOutput = 0;
    private double LeftOutput = 0;

    private double leftOut, rightOut = 0;

    public ZeroClimber(Climber subsystem, double leftOut, double rightOut) {
        // mapping to object passed through parameter
        m_Climber = subsystem;
        m_requirements.add(subsystem);
        this.leftOut = leftOut;
        this.rightOut = rightOut;
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        m_Climber.zero(leftOut, rightOut);
        BrokeLeft = false;
        BrokeRight = false;
        RightOutput = -0.25;
        LeftOutput = -0.25;
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        if (m_Climber.brokeLeft()) {
            LeftOutput = 0;
            m_Climber.resetEncodersLeft();
            BrokeLeft = true;
        }

        if (m_Climber.brokeRight()) {
            RightOutput = 0;
             m_Climber.resetEncodersRight();
            BrokeRight = true;
        }
        m_Climber.setSpeedLeft(LeftOutput);
         m_Climber.setSpeedRight(RightOutput);
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
        return BrokeRight && BrokeLeft;
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
        m_Climber.stop();
    }
}
