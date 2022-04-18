package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import frc.robot.RobotMap;
import frc.robot.SupportingClassess.BallManager;
import frc.robot.sensors.vision.Limelight;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;

public class QuegelCommandGroup extends CommandGroupBase {
    protected final Chassis m_chassis;
    protected final Shooter m_shooter;
    protected final Intake m_intake;
    protected final Magazine m_magazine;

    protected final Limelight m_limelight;
    protected BallManager ball_manager;

    protected final Timer timeSinceReset;

    // command group stuff
    protected final Command[] commands;
    protected char highest = 0;
    protected char lowest = 0;

    protected boolean ranOut = true;

    /**
     * Quegle Command group is a sequential command group that uses a really fast queue
     * @param chassis chassis subsystem
     * @param shooter shooter subsystem
     * @param intake intake subsystem
     * @param magazine magazine subsystem
     * @param limelight limelight singleton
     */
    public QuegelCommandGroup(Chassis chassis, Shooter shooter, Intake intake, Magazine magazine, Limelight limelight) {
        m_chassis = chassis;
        m_shooter = shooter;
        m_intake = intake;
        m_magazine = magazine;

        m_requirements.add(chassis);
        m_requirements.add(shooter);
        m_requirements.add(intake);
        m_requirements.add(magazine);

        m_limelight = limelight;

        timeSinceReset = new Timer();

        commands = new Command[16];
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        timeSinceReset.start();

        commands[lowest & 15].initialize();
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        if (highest != lowest) {
            Command cmd = commands[lowest & 15];
            if (ranOut) {
                cmd.initialize();
            }
            cmd.execute();
            if (cmd.isFinished()) {
                commands[lowest++ & 15].end(false);
                ball_manager.generatePath();
                ranOut = true;
            }
        }
        else {
            lookAround();
            ranOut = true;
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

    }

    /**
     * Periodic method to spin when we don't know what to do
     * Profile should switch when we have something that we can do
     * this is a last resort to be ran
     */
    public void lookAround() {
        m_chassis.driveArcade(0, RobotMap.kSpinnySpeed, false);
        resetOdometery();
    }

    public void resetOdometery() {
        if (m_limelight.hasTrack() && timeSinceReset.hasElapsed(2)) {
            // current problem with this approach is that NAVX will eternally build up error forever, and we will eventually run into a wall
            m_chassis.resetOdometry(
                    new Pose2d(
                            // the vector between the target and the robot
                            new Translation2d(
                                    // this will get rotated about the rotation of the bot
                                    m_limelight.getDistanceToTarget(), m_limelight.getHeading()
                            ),
                            m_chassis.getPose().getRotation()
                    ));
            timeSinceReset.reset();
        }
    }

    @Override
    public void addCommands(Command... commands) {
        for (Command toAdd : commands) {
            addCommand(toAdd);
        }
    }

    public void addCommand(Command toAdd) {
        commands[highest++ & 15] = toAdd;
    }

    public void setBallManager(BallManager ballManager) {
        this.ball_manager = ballManager;
    }
}
