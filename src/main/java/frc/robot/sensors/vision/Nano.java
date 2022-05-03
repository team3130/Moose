package frc.robot.sensors.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotMap;
import frc.robot.SupportingClassess.Ball;
import frc.robot.SupportingClassess.GeneralUtils;

import java.util.concurrent.atomic.AtomicBoolean;

public class Nano implements GeneralUtils {

    protected static final Double[] filler = new Double[0];

    protected static NetworkTable visionTable;

    protected final NetworkTableEntry netx; // x angles

    protected final NetworkTableEntry nety; // y angles

    protected final AtomicBoolean blocking = new AtomicBoolean(false);

    private static final Matrix<N3, N3> rotation = Algebra.Rodrigues(
            Algebra.buildVector(
            Math.toRadians(RobotMap.kLimelightPitch),
            Math.toRadians(RobotMap.kLimelightYaw),
            Math.toRadians(RobotMap.kLimelightRoll)
            )
    );      // Own rotation
    private static final Matrix<N3, N1> translation = Algebra.buildVector(
            RobotMap.kLimelightOffset,
            RobotMap.kLimelightHeight,
            RobotMap.kLimelightLength + RobotMap.targetDistanceOffset
    );


    public Nano(String color) {
        visionTable = NetworkTableInstance.getDefault().getTable("Nano");
        netx = visionTable.getEntry(color + "-x");
        nety = visionTable.getEntry(color + "-y");
    }

    /**
     * Limelight vision tracking pipeline latency.
     * @return Latency in milliseconds
     */
    public double getLatency() {
        return visionTable.getEntry("tl").getDouble(0) + RobotMap.kLimelightLatencyMs;
    }

    /**
     * Read data from the Limelight and update local values
     */
    public Matrix<N3, N1> updateData() {
        return calcPosition(netx.getDoubleArray(filler)[0], nety.getDoubleArray(filler)[0]);
    }


    /**
     * Build a "unit" vector in 3-D and rotate it from camera's
     * coordinates to real (robot's) coordinates
     *
     * @param ax horizontal angle, left is positive
     * @param ay vertical angle, up is positive
     * @return a vector pointing to the direction but with the length = 1
     */
    public Matrix<N3, N1> levelVector(double ax, double ay) {
        // Convert degrees from the vision to unit coordinates
        double ux = Math.tan(Math.toRadians(ax));
        double uy = Math.tan(Math.toRadians(ay));
        // Do two rotations: for LimeLight mount and for robot tilt
        return rotation.times(Algebra.buildVector(ux, uy, 1)); //TODO: make negative
    }

    /**
     * Calculate a position vector based on angles from vision
     *
     * @param ax x value from net tables
     * @param ay y value from net tables
     * @return Resulting vector from the bot
     */
    public Matrix<N3, N1> calcPosition(double ax, double ay) {

        // Find where the vector is actually pointing
        Matrix<N3, N1> v0 = levelVector(ax, ay);

        // Scaling ratio based on the known height of the vision target
        double c = (RobotMap.VISIONTARGETHEIGHT - RobotMap.kLimelightHeight) / v0.get(1, 0);

        // Find the real vector from camera to target
        Matrix<N3, N1> v = v0.times(c);

        // Add the offset of the camera from the turret's origin
        Matrix<N3, N1> a = translation.plus(v);
        // That's the droid we're looking for
        return a;
    }

    public Ball[] updateAll() {
        if (blocking.get()) {
            return new Ball[0];
        }
        blocking.set(true);

        Double[] x = getX();
        Double[] y = getY();

        if (x.length > y.length) {
            Double[] temp = new Double[y.length];
            System.arraycopy(x, 0, temp, 0, temp.length);
            x = temp;

        }
        else if (x.length < y.length) {
            Double[] temp = new Double[x.length];
            System.arraycopy(y, 0, temp, 0, temp.length);
            y = temp;
        }

        Ball[] toReturn = new Ball[x.length];
        int toReturnCounter = 0;

        for (int i = 0; i < x.length; i++) {
            Matrix<N3, N1> result = calcPosition(x[i], y[i]);
            toReturn[toReturnCounter++] = new Ball(result);
        }

        blocking.set(false);

        return toReturn;
    }

    public Double[] getX() {
        return netx.getDoubleArray(filler);
    }

    public Double[] getY() {
        return nety.getDoubleArray(filler);
    }

    public boolean hasBalls() {
        return netx.getDoubleArray(filler).length != 0;
    }

    public Rotation2d getHeading(Matrix<N3, N1> vector) {
        return new Rotation2d(Math.atan(vector.get(0, 0) / vector.get(2, 0)));
    }


    /**
     * Get the ground distance to the target
     *
     * @return distance in inches
     */
    public double getDistanceToBall(Matrix<N3, N1> projection) {
        projection.set(1, 0, 0.0);
        return projection.normF();
    }


    /**
     * Calibrate the tilt angle of the Limelight
     */
    public double calibrate() {
        Matrix<N3, N1> curr = updateData();

        double height = RobotMap.kLimelightHeight;
        double distance = RobotMap.kBallCallibrationDistance;

        return Math.toDegrees(Math.atan2(-height, distance)) - curr.get(2, 0);
    }


    public void outputToShuffleboard() {}
    public void teleopInit() {}
    public void autonInit() {}
    public void disable() {}

    public boolean isNotBlocking() {
        return !blocking.get();
    }
}
