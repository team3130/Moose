package frc.robot.SupportingClassess;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.RobotMap;
import frc.robot.commands.Intake.DeployAndSpintake;
import frc.robot.commands.QuegelCommandGroup;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.sensors.vision.Limelight;
import frc.robot.sensors.vision.Nano;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;

import java.util.Arrays;
import java.util.List;
import java.util.function.Function;
import java.util.function.Predicate;

public class BallManager {
    // 22 is the buffer size (there should only be 11 on the field but meh)
    /**
     * General structure of the array, the quickest ball will have its index cached,
     * the quickest two ball will have its index cached in an array of size two
     * then the overall array will be ordered for thew the quickest path to every known ball
     * This system will have to be dynamic,
     * so that we can find the quickest path for the next n balls into the future
     *
     * This system is designed in this way so that a single async method can handle all the logic
     */
    protected final Ball[] balls = new Ball[16];
    protected int highest = 0;
    protected int lowest = 0;

    protected int quickestOneBall = 0;
    protected int[] quickestTwoBall = new int[] {0, 0};

    // buffer of size 20 for balls to add
    protected final Ball[] toAdd = new Ball[128];
    protected int highestToAddIndex = 0;
    protected int lowestToAddIndex = 0;

    protected final Chassis m_chassis;
    protected final Intake m_intake;
    protected final Magazine m_magazine;
    protected final Shooter m_shooter;

    protected final Limelight m_limelight;

    protected final NetworkTable JetsonNano;
    protected final NetworkTableEntry ballsNano;

    protected final Predicate<Ball> withinFrame;

    protected final PathGeneration m_pathGeneration;
    protected final QuegelCommandGroup commandGroup;

    protected final Function<Trajectory, RamseteCommand> ramseteCommandFactory;
//    protected final Function<Pose2d[], Trajectory> trajectoryCommandFactory;

    protected final TrajectoryConfig config;

    protected final Thread m_managerThread;

    protected final Runnable[] StateMachine;
    protected final char ADD_BALLS = 0;
    protected final char GENERATE_PATH = 1;
    protected final char UPDATE_BALLS = 2;
    protected final char UPDATE_NETWORK_TABLES = 3;
    protected char state = 0;

    protected char updateCounter = 0;

    protected final MagicBox m_magicBox;

    protected final Nano nano;


    public BallManager(Chassis chassis, NetworkTable JetsonNano, PathGeneration pathGeneration, QuegelCommandGroup commandGroup, Chooser chooser, Intake intake, Magazine magazine, Shooter shooter, Limelight limelight, MagicBox magicBox) {
        m_chassis = chassis;
        m_intake = intake;
        m_magazine = magazine;
        m_shooter = shooter;
        m_limelight = limelight;
        this.JetsonNano = JetsonNano;
        ballsNano = JetsonNano.getEntry("balls");

        // the arcTan of the vector between the ball and the bot - direction we are facing, within range of the FOV / 2
        withinFrame = (Ball ball) -> Math.abs(Math.atan2(ball.getY() - m_chassis.getPose().getY(), ball.getX() - m_chassis.getPose().getX()) - m_chassis.getPose().getRotation().getRadians()) <= Math.toRadians(RobotMap.kCameraFOV / 2);

        m_pathGeneration = pathGeneration;
        this.commandGroup = commandGroup;
        commandGroup.setBallManager(this);

        ramseteCommandFactory = chooser.getRamseteCommandFactory();

        config = chooser.getConfig();

        StateMachine = new Runnable[] {this::smartAdd, this::decidePath, this::updateBalls, this::updateNetTables};

        if (NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("isRedAlliance").getBoolean(true)) {
            nano = new Nano("red");
        }
        else {
            nano = new Nano("blue");
        }

        m_managerThread = new Thread(this::manager, "manager");

        this.m_magicBox = magicBox;

    }

    public Ball getQuickestOne() {
        return balls[quickestOneBall];
    }

    public Ball[] getQuickestTwo() {
        return new Ball[] {balls[quickestTwoBall[0]], balls[quickestTwoBall[1]]};
    }

    public Ball[] getPath() {
        // protecting the array
        return balls.clone();
    }

    public void clear() {
        Arrays.fill(balls, null);
    }

    // this is synced with rio, so it merely queues the addition of balls
    public void addBall(Ball... balls) {
        // the slowest thing that gets ran on the main rio thread, but is also pretty unavoidable
        for (Ball ball : balls) {
            toAdd[highestToAddIndex++ & 127] = ball;
        }
    }

    /**
     * If ball in frame and ball no exist, murder ball
     * toAdd should be updated by reading network tables BEFORE this is called
     */
    public void destruct() {
        int toRemoveSize = 0;

        for (int i = (lowest & 15); i < (highest & 15); i++) {
            if (toRemoveSize > 0) {
                balls[i - toRemoveSize] = balls[i];
            }
            // if ball is in frame
            if (withinFrame.test(balls[i])) {
                // see if nano can read it
                boolean safe = false;
                for (int j = lowestToAddIndex & 127; j < (highestToAddIndex & 127); j++) {
                    if (balls[i].equals(toAdd[j])) {
                        safe = true;
                        break;
                    }
                }
                if (!safe) {
                    balls[i] = null;
                    toRemoveSize++;
                }
            }
        }

        if (toRemoveSize > 0) {
            highest -= toRemoveSize;
        }

    }

    // This beafy mofo is gonna cause a threading issue, but unsafe memory operations are funny, so it's worth
    public void smartAdd() {
        // clean up all the arrays
        clean();

        // destination for loop
        for (int i = (lowestToAddIndex & 127); i < (highestToAddIndex & 127); i++) {
            if (toAdd[i] == null) {
                continue;
            }
            boolean duped = false;
            // check if ball already exists in the array
            for (int j = (lowest & 15); j < (highest & 15); j++) {
                if (balls[j].equals(toAdd[i])) {
                    duped = true;
                    break;
                }
            }
            if (duped) {
                toAdd[i] = null;
            }
            else {
                balls[(highest++ & 15)] = toAdd[i];
            }
        }
        updateClosestOneBall();
        updateTwoClosestBalls();
    }

    public boolean ballsExist() {
        return (highest & 15) != (lowest & 15);
    }

    public void clean() {
        destruct();

        // murder all balls past the length (basically a garbage collector)
        for (int i = (highest & 15); i < balls.length; i++) {
            balls[i] = null;
        }
    }

    public void updateClosestOneBall() {
        int closest = 0;
        // this method is extremely slow because it is synchronized
        Pose2d botPose = m_chassis.getPose();
        for (int i = (lowest & 15); i < (highest & 15); i++) {
            if (balls[closest].getDistance(botPose) > balls[i].getDistance(botPose)) {
                closest = i;
            }
        }
        quickestOneBall = closest;
    }
    public void updateTwoClosestBalls() {
        Pose2d botPose = m_chassis.getPose();

        double shortestDistance = Double.MAX_VALUE;

        int firstFastest = 0;
        int secondFastest = 0;

        for (int first = (lowest & 15); first < (highest & 15); first++) {
            for (int second = (lowest & 15); second < (highest & 15); second++) {
                if (first == second) {
                    continue;
                }
                double currDistance = balls[first].getDistance(botPose) + balls[first].getDistance(botPose);
                if (currDistance < shortestDistance) {
                    shortestDistance = currDistance;
                    firstFastest = first;
                    secondFastest = second;
                }
            }
        }
        quickestTwoBall[0] = firstFastest;
        quickestTwoBall[1] = secondFastest;

        state = UPDATE_BALLS;
    }


    public void decidePath() {
        Pose2d start = commandGroup.getCurr().getPoseGoingTo();

        double firstBallX = balls[quickestTwoBall[0]].getX();
        double firstBallY = balls[quickestTwoBall[0]].getY();

        double secondBallX = balls[quickestTwoBall[1]].getX();
        double secondBallY = balls[quickestTwoBall[1]].getY();

        Rotation2d rotationBetweenBall1AndBall2 =
                new Rotation2d(Math.atan2(secondBallY - firstBallY, secondBallX - firstBallX));

        Pose2d firstBall = new Pose2d(
                firstBallX,
                firstBallY,
                new Rotation2d((rotationBetweenBall1AndBall2.getRadians() + start.getRotation().getRadians()) / 2)
        );


        Rotation2d rotationBetweenSecondBallAndTarget =
                new Rotation2d(Math.atan2(RobotMap.kTargetPose.getY() - secondBallY, RobotMap.kTargetPose.getX() - secondBallX));

        Pose2d secondBall = new Pose2d(secondBallX, secondBallY, new Rotation2d((rotationBetweenSecondBallAndTarget.getRadians() + firstBall.getRotation().getRadians()) / 2.0));


        CommandBase deployAndSpintake = new DeployAndSpintake(m_intake, m_magazine, 1);

        RamseteCommand goToBalls = ramseteCommandFactory.apply(
                TrajectoryGenerator.generateTrajectory(
                        List.of(
                                start,
                                firstBall,
                                secondBall
                        ),
                        config
                )
        );

        Trajectory toShootTrajectory = m_pathGeneration.getShootPath(secondBall);
        RamseteCommand goToShoot = ramseteCommandFactory.apply(toShootTrajectory);

        CommandBase shoot = new Shoot(m_shooter, m_magazine, m_chassis, m_limelight);

        commandGroup.addCommand(new Event(secondBall, new ParallelDeadlineGroup(goToBalls, deployAndSpintake), EventType.GOING_TO_TWO_BALLS));
        commandGroup.addCommand(new Event(toShootTrajectory.getStates().get(toShootTrajectory.getStates().size() - 1).poseMeters, goToShoot, EventType.GOING_TO_SHOOT));
        commandGroup.addCommand(new Event(toShootTrajectory.getStates().get(toShootTrajectory.getStates().size() - 1).poseMeters, shoot, EventType.SHOOTING));

        state = UPDATE_BALLS;
    }

    @SuppressWarnings("InfiniteLoopStatement")
    public void manager() {
        while (true) {
            StateMachine[state].run();
        }
    }

    /**
     * This isn't the slow one
     */
    public void generatePath() {
        state = GENERATE_PATH;
    }

    public void updateBalls() {
        Pose2d chassisPos = m_chassis.getPose();
        double[] pos = new double[] {chassisPos.getX(), chassisPos.getY()};
        Ball[] ballArr = nano.updateAll();

        for (int i = 0; i < ballArr.length; i++) {
            ballArr[i].transformTo(pos);
        }

        addBall(ballArr);

        updateCounter++;

        if ((updateCounter & 1) == 1) {
            state = ADD_BALLS;
        }
        if ((updateCounter & 3) == 3) {
            state = UPDATE_NETWORK_TABLES;
        }
    }

    public void updateNetTables() {
        double[] x = new double[balls.length];
        double[] y = new double[balls.length];
        for (int i = (lowest & 15); i < (highest & 15); i++) {
            x[i] = balls[i].getX();
            y[i] = balls[i].getY();
        }
        if (m_magicBox.updateBalls(x, y)) {
            state = UPDATE_BALLS;
        }
    }

}
