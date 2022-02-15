package frc.robot.team3130.SupportingClasses;

public class Coordinates {
    // field coords
    private double xPos;
    private double yPos;
    private double zPos;

    // final constants
    private static final double X_CONSTANT = 1; // TODO: determine this
    private static final double Y_CONSTANT = 1; // TODO: determine this
    private static final double HEIGHT_CONSTANT = 1; // TODO: determine this
    

    public Coordinates(int xCam, int yCam, double ballRad, double botX, double botY, double botZ, double botYaw, double botPitch) {
        double[] coords = translation(rotation(dilation(xCam, yCam, ballRad), botYaw, botPitch), botX, botY, botZ);
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
        double[] coords = {
                xCam * X_CONSTANT * HEIGHT_CONSTANT / ballRad,
                HEIGHT_CONSTANT / ballRad,
                yCam * Y_CONSTANT * HEIGHT_CONSTANT / ballRad,
        };
        return coords;

    }

    /**
     * Rotation matrix
     * Rotates the axis by @param botAngle radians
     */
    private static double[] rotation(double[] coords, double yaw, double pitch) {
        double xPos = coords[0];
        double yPos = coords[1];
        double zPos = coords[2];

        //Correct for pitch
        coords[1] = Math.cos(pitch) * yPos - Math.sin(pitch) * zPos;
        coords[2] = Math.sin(pitch) * yPos + Math.cos(pitch) * zPos;

        //Correct for yaw
        coords[0] = Math.cos(yaw) * xPos - Math.sin(yaw) * yPos;
        coords[1] = Math.sin(yaw) * xPos + Math.cos(yaw) * yPos;
        
        return coords;

    }

    /**
     * Translation matrix
     * Shifts the origin of the coordinate plane by @param botX 
     */
    private static double[] translation(double[] coords, double botX, double botY, double botZ) {
        coords[0] = coords[0] + botX;
        coords[1] = coords[1] + botY;
        coords[2] = coords[2] + botZ;
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