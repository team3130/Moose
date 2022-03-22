package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.vision.Limelight;
import frc.robot.sensors.vision.WheelSpeedCalculations;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

public class Shoot extends CommandBase {
    // defining an instance to be used throughout the command and to be instantiated in the constructor of type parameter
    private final Shooter m_shooter;
    private final Hood m_hood;
    private boolean hitSpeed = false;
    private Limelight limelight;
    private WheelSpeedCalculations shooterCurve;
    private WheelSpeedCalculations hoodCurve;

    public Shoot(Shooter subsystem, Hood hood, Limelight limelight) {
        //mapping to object passed through parameter
        m_shooter = subsystem;
        m_hood = hood;
        m_requirements.add(subsystem);
        m_requirements.add(hood);

        this.limelight = limelight;
        this.shooterCurve = m_shooter.getShooterCurve();
        hoodCurve = m_hood.getWinchCurve();

    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {}

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        // Find the flywheel speed
        if (!limelight.hasTrack()){
            m_shooter.setFlywheelSpeed(m_shooter.getRPM());
        }
        else {
            double x = limelight.getDistanceToTarget();
            if (5 <= x) {
                m_shooter.setFlywheelSpeed(shooterCurve.getSpeed(x));
                m_hood.toPos(hoodCurve.getSpeed(x));
            }
            if (m_shooter.canShoot() && m_hood.canShoot(hoodCurve.getSpeed(x))) {
                m_shooter.feedIndexer();
                hitSpeed = true;
            }
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
        return hitSpeed && m_shooter.getRPM() <= m_shooter.getFlywheelSetSpeed() * 0.75; // if flywheel dropped 75% of its speed
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
        m_shooter.setIndexerPercent(0);
    }
}
