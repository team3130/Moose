package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.SupportingClassess.BallManager;
import frc.robot.SupportingClassess.Chooser;
import frc.robot.sensors.vision.Limelight;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;

public class TeleAuto extends CommandBase {
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

    protected final Runnable[] functions;

    protected int state = 0;

    protected final NetworkTable JetsonNano;
    protected final NetworkTableEntry JetsonBalls;

    protected Pose2d targetPos;

    protected final Timer timeSinceLook;

    public TeleAuto(Chassis chassis, Shooter shooter, Intake intake, Magazine magazine, Limelight limelight, BallManager ballManager, Chooser chooser, NetworkTable JetsonNano) {
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

        functions = new Runnable[] {this::lookAround, this::goToBall, this::driveToShoot};

        this.JetsonNano = JetsonNano;
        JetsonBalls = JetsonNano.getEntry("balls");

        targetPos = new Pose2d();

        timeSinceLook = new Timer();
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        timeSinceLook.start();

    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        // hashmaps are too slow...
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
     */
    public void lookAround() {
        m_chassis.driveArcade(0, RobotMap.kSpinnySpeed, false);
        if (m_limelight.hasTrack() && timeSinceLook.hasElapsed(2)) {
            // current problem with this approach is that NAVX will eternally build up error forever, and we will eventually run into a wall
            //
            targetPos = new Pose2d(
                    m_chassis.getPose().getTranslation().plus(
                            // the vector between the target and the bot
                            new Translation2d(
                                    m_limelight.getDistanceToTarget(), m_limelight.getHeading().plus(m_chassis.getPose().getRotation())
                            )
                    ),
                    m_chassis.getPose().getRotation()
            );
            timeSinceLook.reset();
        }
        if () {}

    }

    public void goToBall() {

    }

    public void driveToShoot() {

    }
}
