package frc.robot.team3130.SupportingClasses;

public class Coordinates {
    private double x;
    private double y;
    private double z;
    private final double X_CONSTANT = 1;    //TODO: determine this
    private final double Y_CONSTANT = 1;    //TODO: determine this
    private final double Z_CONSTANT = 1;    //TODO: determine this

    Coordinates(double x, double y, double dia) {
        convert(x, y, dia);
        // convertSpherical(x, y, dia);
    }

    private void convert(double x, double y, double dia) {
        this.z = Z_CONSTANT / dia;
        this.x = x * z * X_CONSTANT;
        this.y = y * z * Y_CONSTANT;

    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getZ() {
        return z;
    }
}