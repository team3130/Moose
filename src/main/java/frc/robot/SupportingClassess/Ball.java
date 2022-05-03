package frc.robot.SupportingClassess;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.RobotMap;

public class Ball {
    protected final double[] pose;

    public Ball(double x, double y) {
        pose = new double[] {x, y};
    }

    public Ball(Matrix<N3, N1> mat) {
        pose = new double[] {mat.get(0, 0), mat.get(2, 0)};
    }


    public double getDistance(double[] pose) {
        return getDistance(pose[0], pose[1]);
    }

    public double getDistance(Pose2d pose) {
        return getDistance(pose.getX(), pose.getY());
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

    public double[] getPose() {
        return pose;
    }

    public double getX() {
        return pose[0];
    }

    public double getY() {
        return pose[1];
    }
    
    public boolean equals(double x, double y) {
        return Math.sqrt(Math.pow(pose[0] - x, 2) + Math.pow(pose[1] - y, 2)) <= RobotMap.ballPositionError;
    }

    public boolean equals(Ball other) {
        return Math.sqrt(Math.pow(pose[0] - other.pose[0], 2) + Math.pow(pose[1] - other.pose[1], 2)) <= RobotMap.ballPositionError;
    }

    public boolean equals(double[] pose) {
        return Math.sqrt(Math.pow(this.pose[0] - pose[0], 2) + Math.pow(this.pose[1] - pose[1], 2)) <= RobotMap.ballPositionError;
    }
}
