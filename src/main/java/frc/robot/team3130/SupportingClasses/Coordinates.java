package frc.robot.team3130.SupportingClasses;

public class Coordinates {
    private double xPos;
    private double yPos;
    private double zPos;
    private final double X_CONSTANT = 1; // TODO: determine this
    private final double Y_CONSTANT = 1; // TODO: determine this
    private final double Z_CONSTANT = 1; // TODO: determine this

    Coordinates(double x, double y, double diameter) {
        convert(x, y, diameter);
    }

    private void convert(double x, double y, double diameter) {
        zPos = Z_CONSTANT / diameter;
        xPos = x * zPos * X_CONSTANT;
        yPos = y * zPos * Y_CONSTANT;

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