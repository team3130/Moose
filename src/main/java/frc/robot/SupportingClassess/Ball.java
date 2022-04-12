package frc.robot.SupportingClassess;

public class Ball {
    protected final double[] pose;

    public Ball(double x, double y) {
        pose = new double[] {x, y};
    }


    public double getDistance(double[] pose) {
        return getDistance(pose[0], pose[1]);
    }

    /**
     * Standard Euclidean distance
     * (yes this is slow as duck)
     * @param x x of comparing pose
     * @param y y of comparing pose
     * @return euclidean distance
     */
    public double getDistance(double x, double y) {
        return Math.sqrt(Math.pow(pose[0] - x, 2) + Math.pow(pose[1] - y, 2));
    }
}
