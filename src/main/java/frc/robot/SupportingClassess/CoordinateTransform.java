package frc.robot.SupportingClassess;

import edu.wpi.first.math.util.Units;

public class CoordinateTransform {
    private double xPos;
    private double yPos;
    private double zPos;

    // Camera and Field Specs
    private static final double BALL_RADIUS = Units.inchesToMeters(4.75); // Jerry fuck you
    private static final double CAMERA_X = 3280; // pixels
    private static final double CAMERA_Y = 2464; // pixels
    private static final double CAMERA_HORIZONTAL_FOV = 62.2; // degrees
    private static final double CAMERA_VERTICAL_FOV = 48.8;

    // Calculation Constants
    private static final double X_CONSTANT = 2 * BALL_RADIUS; 
    private static final double Y_CONSTANT = 2 * BALL_RADIUS; 
    private static final double Z_CONSTANT = BALL_RADIUS * Math.tan(Math.toRadians(90 - CAMERA_HORIZONTAL_FOV / 2)) * CAMERA_X;
    private static final double Z_CONSTANT2 = BALL_RADIUS * Math.tan(Math.toRadians(90 - CAMERA_VERTICAL_FOV / 2)) * CAMERA_Y; //This should be extremely close to the first one
    private static final double CAMERA_PITCH = 0; //TODO: get values

    public CoordinateTransform(int xCam, int yCam, double ballRad, double botYaw) {
        double[] coords = rotation(dilation(xCam, yCam, ballRad), botYaw, CAMERA_PITCH);
        xPos = coords[0];
        yPos = coords[1];
        zPos = coords[2];
    }

    /**
     * Finds the ball's distance from the camera and creates (X,Y,Z) coords
     * with the camera position at the origin.
     * assume (0,0) is at the center of the camera image.
     */
    private static double[] dilation(double xCam, double yCam, double ballRad) {
        return new double[]{
                xCam * X_CONSTANT / ballRad,
                yCam * Y_CONSTANT / ballRad,
                Z_CONSTANT / ballRad,
        };
    }

    /**
     * Rotation matrix
     * Rotates the axis by @param botAngle radians
     */
    private static double[] rotation(double[] coords, double yaw, double pitch) {

        // Correct for pitch
        coords[1] = Math.cos(pitch) * coords[1] - Math.sin(pitch) * coords[2];
        coords[2] = Math.sin(pitch) * coords[1] + Math.cos(pitch) * coords[2];

        // Correct for yaw
        coords[0] = Math.cos(yaw) * coords[0] - Math.sin(yaw) * coords[2];
        coords[2] = Math.sin(yaw) * coords[0] + Math.cos(yaw) * coords[2];

        return coords;
    }

    public double getX() {
        return xPos;
    }

    public double getY() {
        return yPos;
    }

    public double getZ() {
        return zPos;
    }
}