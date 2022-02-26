package frc.robot.team3130.SupportingClasses;

import java.lang.Math;

import edu.wpi.first.math.geometry.Translation2d;

public class Node {
    private double xPos;
    private double yPos;

    public Node(double xPos, double yPos) {
        this.xPos = xPos;
        this.yPos = yPos;
    }

    public double distance(Node other) {
        return Math.sqrt(Math.pow(this.xPos - other.xPos, 2) + Math.pow(this.yPos - other.yPos, 2));
    }

    public double angleInRadians(Node to) {
        return Math.atan2(to.yPos - this.yPos, to.xPos - this.xPos);
    }

    public double angleInRadians(Node to, Node from) {
        return (from.angleInRadians(this) + this.angleInRadians(to)) / 2;
    }

    public double getX() {
        return xPos;
    }

    public double getY() {
        return yPos;
    }

    public Translation2d toTranslation2d() {
        return new Translation2d(xPos, yPos);
    }
}