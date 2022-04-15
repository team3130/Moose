package frc.robot.SupportingClassess;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.RobotMap;
import frc.robot.subsystems.Chassis;

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
    protected final Ball[] balls = new Ball[22];
    protected int highest = 0;

    protected int quickestOneBall = 0;
    protected int[] quickestTwoBall = new int[] {0, 0};

    // buffer of size 20 for balls to add
    protected final Ball[] toAdd = new Ball[20];
    protected int highestToAddIndex = 0;
    protected int lowestToAddIndex = 0;

    protected final Chassis m_chassis;

    protected final NetworkTable JetsonNano;
    protected final NetworkTableEntry ballsNano;

    protected final Predicate<Ball> withinFrame;

    public BallManager(Chassis chassis, NetworkTable JetsonNano) {
        m_chassis = chassis;
        this.JetsonNano = JetsonNano;
        ballsNano = JetsonNano.getEntry("balls");

        withinFrame = (Ball ball) -> Math.abs(Math.atan2(ball.getY() - m_chassis.getPose().getY(), ball.getX() - m_chassis.getPose().getX()) - m_chassis.getPose().getRotation().getRadians()) <= Math.toRadians(RobotMap.kCameraFOV / 2);
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
        // when you're making a destructor in Java
        for (Ball ball : balls) {
            ball = null;
        }
    }

    // this is synced with rio, so it merely queues the addition of balls
    public void addBall(Ball... balls) {
        // the slowest thing that gets ran on the main rio thread, but is also pretty unavoidable
        for (Ball ball : balls) {
            toAdd[highestToAddIndex++] = ball;
        }
    }
    public void addAvailable() {

    }

    /**
     * If ball in frame and ball no exist, murder ball
     * toAdd should be updated by reading network tables BEFORE this is called
     */
    public void destruct() {
        int toRemoveSize = 0;

        for (int i = 0; i < highest; i++) {
            if (toRemoveSize > 0) {
                balls[i - toRemoveSize] = balls[i];
            }
            // if ball is in frame
            if (withinFrame.test(balls[i])) {
                // see if nano can read it
                boolean safe = false;
                for (int j = 0; j < toAdd.length; j++) {
                    if (balls[i].equals(toAdd[j])) {
                        safe = true;
                        break;
                    }
                }
                if (safe) {
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
        updateToAdd();
        destruct();

        Ball[] collection = new Ball[22];

        int collectionHighest = 0;

        for (int i = 0; i < highest; i++) {
            if (balls[i] != null) {
                collection[collectionHighest] = balls[i];
            }
        }

        highest = collectionHighest;

        if (highestToAddIndex != 0) {
            System.arraycopy(toAdd, highest, collection, highest, highestToAddIndex);
        }

        highest += highestToAddIndex;

        for (int i = 0; i < highest; i++) {
            balls[i] = collection[i];
        }
    }

    public boolean ballsExist() {
        return highest <= 0;
    }

    public void clean() {
        updateToAdd();
        destruct();

        // murder all balls past the length (basically a garbage collector)
        for (int i = highest; i < balls.length; i++) {
            balls[i] = null;
        }
    }

    public void updateToAdd() {
        double[] nanoBalls = ballsNano.getDoubleArray(new double[0]);
        int nanoIndex = 0;
        int toAddIndex = highestToAddIndex;
        while (nanoIndex < nanoBalls.length) {
            double x = nanoBalls[nanoIndex++];
            double y = nanoBalls[nanoIndex++];
            toAdd[toAddIndex++ % toAdd.length] = new Ball(x, y);
        }
    }
}
