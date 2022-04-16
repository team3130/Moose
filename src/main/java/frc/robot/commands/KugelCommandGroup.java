package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import frc.robot.RobotMap;
import frc.robot.SupportingClassess.BallManager;
import frc.robot.SupportingClassess.Chooser;
import frc.robot.sensors.vision.Limelight;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;

import java.util.ArrayDeque;

public class KugelCommandGroup extends CommandGroupBase {
    protected final Chassis m_chassis;
    protected final Shooter m_shooter;
    protected final Intake m_intake;
    protected final Magazine m_magazine;

    protected final Limelight m_limelight;
    protected final BallManager m_ballManager;
    protected final Chooser m_chooser;


    // enums are too bulky for this, indices are superior
    protected final char LOOKING_AROUND = 0;
    protected final char GOING_TO_BALL = 1;
    protected final char GOING_TO_SHOOT = 2;
    protected final char SHOOTING = 3;

    protected final Runnable[] functions;

    protected int state = 0;

    protected CommandBase cmd;

    protected final NetworkTable JetsonNano;
    protected final NetworkTableEntry JetsonBalls;
    protected final Timer timeSinceReset;

    // threading stuff
    protected Thread thread;

    // command group stuff
    protected final ArrayDeque<CommandBase> commands;
    protected boolean ranOut = true;

    public KugelCommandGroup(Chassis chassis, Shooter shooter, Intake intake, Magazine magazine, Limelight limelight, BallManager ballManager, Chooser chooser, NetworkTable JetsonNano) {
        m_chassis = chassis;
        m_shooter = shooter;
        m_intake = intake;
        m_magazine = magazine;

        m_limelight = limelight;
        m_ballManager = ballManager;
        m_chooser = chooser;

        m_requirements.add(chassis);
        m_requirements.add(shooter);
        m_requirements.add(intake);
        m_requirements.add(magazine);

        functions = new Runnable[] {this::lookAround, this::goingToBall, this::drivingToShoot, this::shooting};

        this.JetsonNano = JetsonNano;
        JetsonBalls = JetsonNano.getEntry("balls");

        timeSinceReset = new Timer();

        commands = new ArrayDeque<>();

        thread = new Thread(this::manager);
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        timeSinceReset.start();

        if (!commands.isEmpty()) {
            commands.peekFirst().initialize();
        }
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        if (!commands.isEmpty()) {
            CommandBase cmd = commands.peekFirst();
            if (ranOut) {
                cmd.initialize();
            }
            cmd.execute();
            if (cmd.isFinished()) {
                commands.pop().end(false);
                ranOut = true;
            }
        }
        else {
            ranOut = true;
        }
        functions[state].run();
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

    public void goingToBall() {
        if ()
    }

    public void drivingToShoot() {

    }

    public void shooting() {
        resetOdometery();
    }

    @Override
    public void addCommands(Command... commands) {

    }

    @SuppressWarnings("InfiniteLoopStatement")
    public void manager() {
        while (true) {
            functions[state].run();
        }
    }

    protected synchronized CommandBase getFirst() {
        return commands.peekFirst();
    }
}
