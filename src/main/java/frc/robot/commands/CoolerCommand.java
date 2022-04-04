package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.ChassisCooler;


public class CoolerCommand extends CommandBase {
    // defining an instance to be used throughout the command and to be instantiated in the constructor of type parameter
    private final ChassisCooler m_ChassisCooler;

    private final Chassis m_Chassis;

    private final Timer TimeSince;
    private final Timer FiringTime;

    public CoolerCommand(ChassisCooler subsystem, Chassis chassis) {
        //mapping to object passed through parameter
        m_ChassisCooler = subsystem;
        m_Chassis = chassis;
        m_requirements.add(subsystem);
        TimeSince = new Timer();
        FiringTime = new Timer();
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        m_ChassisCooler.SetCooler(false);
        TimeSince.reset();
        FiringTime.reset();
        TimeSince.start();

    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        if (m_Chassis.motorTemp() > RobotMap.MaxMotorTemp )  {
            m_ChassisCooler.SetCooler(true);
            FiringTime.start();
        }
        else if (m_Chassis.motorTemp() < RobotMap.MaxMotorTemp - 5)    {
            m_ChassisCooler.SetCooler(false);
            FiringTime.stop();
            FiringTime.reset();
        }
    }


   /* if (TimeSince.hasElapsed(0.5)) {
        m_ChassisCooler.SetCooler(true);
        FiringTime.start();
        TimeSince.reset();
        }

    if (FiringTime.hasElapsed(0.5)) {
        m_ChassisCooler.SetCooler(false);
        FiringTime.reset();
    }*/



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
        return FiringTime.hasElapsed(20);
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
        m_ChassisCooler.SetCooler(false);
        FiringTime.stop();
    }
}
