package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.vision.Limelight;
import frc.robot.sensors.vision.WheelSpeedCalculations;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;

public class SetFlywheelRPM extends CommandBase {

    private boolean reloading = false;
    // defining an instance to be used throughout the command and to be instantiated in the constructor of type parameter
    private final Shooter m_shooter;
    private Limelight limelight;
    private Magazine m_magazine;
    private WheelSpeedCalculations shooterCurve;
    private Chassis m_chassis;

    private Timer timerShoot;
    private double timeShoot = 0.2;

    private final Timer timerSpin = new Timer();
    private final double timeSpin = 0.5;

    public SetFlywheelRPM(Shooter subsystem, Magazine magazine, Chassis chassis, Limelight limelight) {
        //mapping to object passed through parameter
        m_shooter = subsystem;
        m_requirements.add(subsystem);
        m_chassis = chassis;
        m_requirements.add(chassis);

        m_magazine = magazine;
        m_requirements.add(magazine);

        this.limelight = limelight;

        shooterCurve = m_shooter.getShooterCurve();

        timerShoot = new Timer();
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        limelight.setLedState(true);
        m_shooter.updatePID();
        m_shooter.setFlywheelSpeeds(m_shooter.getSpeedFromShuffleboard());

        timerSpin.reset();
        timerSpin.start();
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        m_shooter.setFlywheelSpeed(m_shooter.getSpeedFromShuffleboard());
        if (m_shooter.canShoot()) {
            m_shooter.setIndexerPercent(0.6);
            timerShoot.reset();
            timerShoot.start();
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
        m_shooter.stopAll();
        m_magazine.stopAll();
        timerShoot.stop();
        timerSpin.stop();
    }
}
