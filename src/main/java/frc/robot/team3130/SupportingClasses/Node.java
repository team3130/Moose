package frc.robot.team3130.SupportingClasses;

import java.lang.Math;

public abstract class Node {
    private double xPos;
    private double yPos;
    
    public Node(double xPos, double yPos) {
        this.xPos = xPos;
        this.yPos = yPos;
    }

    public double distance(Node other) {
        return Math.sqrt(Math.pow(this.xPos - other.xPos, 2) + Math.pow(this.yPos - other.yPos, 2));
    }

    public double getX() {
        return xPos;
    }

    public double getY() {
        return yPos;
    }

    public abstract String nodeType();
}