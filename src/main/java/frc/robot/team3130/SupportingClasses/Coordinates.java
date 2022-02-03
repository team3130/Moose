package frc.robot.team3130.SupportingClasses;

public class Coordinates {
    private double xPos;
    private double yPos;
    private double zPos;
    private final double X_CONSTANT = 1; // TODO: determine this
    private final double Y_CONSTANT = 1; // TODO: determine this
    private final double Z_CONSTANT = 1; // TODO: determine this

    Coordinates(double x, double y, double diameter, double botX, double botY, double botAngle) {
        translation(rotation(convert(x, y, diameter), botAngle), botX, botY);
    }

    private static double[] convert(double x, double y, double diameter) {
        double[] coords = {
                x * X_CONSTANT * Z_CONSTANT / diameter,
                y * Y_CONSTANT * Z_CONSTANT / diameter,
                Z_CONSTANT / diameter,
        };
        return coords;

    }

    private static double[] rotation(double[] coords, double botAngle) {
        double x = coords[0];
        double y = coords[1];
        coords[0] = Math.cos(botAngle) * x - Math.sin(botAngle) * y;
        coords[1] = Math.sin(botAngle) * x + Math.cos(botAngle) * y;
        return coords;

    }

    private static void translation(double[] coords, double botX, double botY) {
        xPos = coords[0] + botX;
        yPos = coords[1] + botY;
        zPos = coords[2];
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