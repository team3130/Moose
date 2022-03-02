package frc.robot.sensors.vision;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.RobotMap;
import frc.robot.subsystems.Chassis;

//TODO: Make this not static and pass turret object as a paraneter to be used here

public class Limelight {
    //Instance Handling
    private static Limelight m_pInstance;

    public static Limelight GetInstance() {
        if (m_pInstance == null) m_pInstance = new Limelight();
        return m_pInstance;
    }

    NetworkTable visionTable;

    private NetworkTableEntry tv; // Whether the limelight has any valid targets (0 or 1)
    private NetworkTableEntry tx; // x angle offset from crosshair, range of -27 to 27
    private NetworkTableEntry ty; // y angle offset from crosshair, range of -20.5 to 20.5
    private NetworkTableEntry ta; // area of contour bounding box
    private NetworkTableEntry ts; // Skew or rotation (-90 degrees to 0 degrees)


    private MedianFilter txFilter;
    private MedianFilter tyFilter;
    private MedianFilter tsFilter;
    private double x_targetOffsetAngle;
    private double y_targetOffsetAngle;
    private double area;
    private double skew;

    private Matrix<N3, N3> rotation;      // Own rotation
    private Matrix<N3, N1> translation;   // Own translation
    private Matrix<N3, N1> realVector;
    private Matrix<N3, N1> sideVector;

    protected Limelight() {
        visionTable = NetworkTableInstance.getDefault().getTable("limelight");
        tv = visionTable.getEntry("tv");
        tx = visionTable.getEntry("tx");
        ty = visionTable.getEntry("ty");
        ta = visionTable.getEntry("ta");
        ts = visionTable.getEntry("ts");

        txFilter = new MedianFilter(RobotMap.kLimelightFilterBufferSize);
        tyFilter = new MedianFilter(RobotMap.kLimelightFilterBufferSize);
        tsFilter = new MedianFilter(RobotMap.kLimelightFilterBufferSize);
        x_targetOffsetAngle = 0.0;
        y_targetOffsetAngle = 0.0;
        area = 0.0;
        skew = 0.0;

        Matrix<N3, N1> rVec = Algebra.buildVector(
                Math.toRadians(RobotMap.kLimelightPitch),
                Math.toRadians(RobotMap.kLimelightYaw),
                Math.toRadians(RobotMap.kLimelightRoll)
        );
        rotation = Algebra.Rodrigues(rVec);
        translation = Algebra.buildVector(
                RobotMap.kLimelightOffset,
                RobotMap.kLimelightHeight,
                RobotMap.kLimelightLength
        );
        realVector = Algebra.buildVector(0, 0, 0);
        sideVector = Algebra.buildVector(0, 0, 0);
    }

    public double getTx() {
        return tx.getDouble(0.0);
    }

    public double getTy() {
        return ty.getDouble(0.0);
    }

    public double getArea() {
        return ta.getDouble(0.0);
    }

    public double getSkew() {
        return ts.getDouble(0.0);
    }

    /**
     * Limelight vision tracking pipeline latency.
     * @return Latency in milliseconds
     */
    public double getLatency(){
        return visionTable.getEntry("tl").getDouble(0) + RobotMap.kLimelightLatencyMs;
    }

    /**
     * Read data from the Limelight and update local values
     */
    public void updateData() {
        x_targetOffsetAngle = txFilter.calculate(getTx());
        y_targetOffsetAngle = tyFilter.calculate(getTy());
        area = getArea();
        skew = tsFilter.calculate(getSkew());
        realVector = calcPosition(x_targetOffsetAngle, y_targetOffsetAngle);

        // A side vector is a point somewhere on the line that connects
        // the two top corners of the target, i.e. the top edge.
        // Doesn't have to be a corner necessarily.
        double realSkew = Math.toRadians(skew < -45 ? skew + 90 : skew);
        double side_x = x_targetOffsetAngle + Math.cos(realSkew);
        double side_y = y_targetOffsetAngle + Math.sin(realSkew);
        sideVector = calcPosition(side_x, side_y);
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
        return rotation.times(Algebra.buildVector(ux, uy, 1));
    }

    /**
     * Calculate a position vector based on angles from vision
     *
     * @param ax Horizontal Offset From Crosshair To Target
     * @param ay Vertical Offset From Crosshair To Target
     * @return resulting vector from the Turret's origin to the target
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

    /**
     * If the Limelight has a target track
     *
     * @return true if Limelight has targets
     */
    public boolean hasTrack() {
        return tv.getDouble(0.0) == 1.0;
    }

    public double getTargetRotationTan() {
        Matrix<N3,N1> edge = sideVector.minus(realVector);
        return -edge.get(2, 0) / edge.get(0, 0);
    }

    public Matrix<N3,N1> getInnerTarget() {
        double depth = 29.25;
        double alpha = Math.atan(getTargetRotationTan());
        return Algebra.buildVector(
            depth * Math.sin(alpha),
            0,
            depth * Math.cos(alpha));
    }

    /**
     * Get the horizontal angle error to target
     *
     * @return angle in degrees
     */
    public double getDegHorizontalError() {
        // realVector is calculated in updateData that should be called before doing this
        // The horizontal error is an angle between the vector's projection on the XZ plane
        // and the Z-axis which is where the turret is always facing.
        double alpha = Math.toDegrees(Math.atan2(realVector.get(0, 0), realVector.get(2, 0)));
        Matrix<N3,N1> inner = getInnerTarget();
        double yawAdj = 0; // GetInstance().getYawAdjustment();
        System.out.println("Yaw Adjustment: " + yawAdj);

//        System.out.println("Inner Goal offset: " + Math.toDegrees(Math.atan2(inner.get(0, 0), inner.get(2, 0))));

        // TODO: Explain why is this negative
        // If rotation of the target is greater than this many inches along the edge
        // of the outer goal (approx) forget about the inner goal
        if (Math.abs(inner.get(0, 0)) > 5) return -1.0 * alpha + yawAdj;

        // Otherwise add the inner goal's vector to the target vector
        // to obtain a new aiming angle
        Matrix<N3,N1> adjustedVec = realVector.minus(inner);
        return -1.0 * Math.toDegrees(Math.atan2(adjustedVec.get(0, 0), adjustedVec.get(2, 0))) + yawAdj;
    }

    /**
     * Get the ground distance to the target
     *
     * @return distance in inches
     */
    public double getDistanceToTarget() {
        if (Limelight.GetInstance().hasTrack()) {
            Matrix<N3, N1> projection = realVector.copy();
            projection.set(1, 0, 0.0);
            return projection.normF();
        } else {
            return 0.0;
        }
    }


    /**
     * Calibrate the tilt angle of the Limelight
     */
    public double calibrate() {
        updateData();

        double height = RobotMap.VISIONTARGETHEIGHT - RobotMap.kLimelightHeight;
        double distance = RobotMap.kLimelightCalibrationDist;

        return Math.toDegrees(Math.atan2(height, distance)) - y_targetOffsetAngle;
    }

    /**
     * Turn the LEDs of the Limelight on or off
     *
     * @param isOn true means on
     */
    public void setLedState(boolean isOn) {
        if (isOn) {
            visionTable.getEntry("ledMode").setNumber(3);
        } else {
            visionTable.getEntry("ledMode").setNumber(1);
        }
    }

    /**
     * Set the current vision tracking pipeline of the Limelight
     *
     * @param pipelineNumber the id of the pipeline
     */
    public void setPipeline(double pipelineNumber) {
        visionTable.getEntry("pipeline").setNumber(pipelineNumber);
        /*
        How to set a parameter value:
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("<PUT VARIABLE NAME HERE>").setNumber(<TO SET VALUE>);
        */
    }

    public double getYawAdjustment(){
        double distance = getDistanceToTarget();
        double yawCalc = 0.0;
        if(distance >= 220){
        yawCalc = RobotMap.kLimelightYaw *
                Math.min(1.0, Math.max(0.0, (distance - 190)) / (290 - 190));
        }
        return yawCalc;
    }


    public void outputToShuffleboard(Chassis chassis) {
        Limelight o = GetInstance();
        SmartDashboard.putNumber("Limelight X", o.x_targetOffsetAngle);
        SmartDashboard.putNumber("Limelight Y", o.y_targetOffsetAngle);
        SmartDashboard.putNumber("Limelight Distance", o.getDistanceToTarget());
        SmartDashboard.putNumber("Limelight Area", o.area);
        SmartDashboard.putBoolean("Limelight Has Target", o.hasTrack());
        SmartDashboard.putNumber("Limelight mounting angle", o.calibrate());
        SmartDashboard.putNumber("Limelight Target Rotation", o.getTargetRotationTan());
    }

}
