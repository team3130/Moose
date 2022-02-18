package frc.robot.team3130.SupportingClasses;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.lang.Math;

public class Node {
    private Pose2d pose;

    public Node(double xPos, double yPos) {
        pose = new Pose2d(xPos, yPos, new Rotation2d());
    }

    public Node(double xPos, double yPos, Rotation2d rotation2d) {
        pose = new Pose2d(xPos, yPos, rotation2d);
    }

    public double distance(Node other) {
        return Math.sqrt(
                Math.pow(this.pose.getX() - other.pose.getX(), 2) + Math.pow(this.pose.getY() - other.pose.getY(), 2));
    }

    public double angleInRadians(Node to) {
        return Math.atan2(to.pose.getY() - this.pose.getY(), to.pose.getX() - this.pose.getX());
    }

    public double angleInRadians(Node to, Node from) {
        return (from.angleInRadians(this) + this.angleInRadians(to)) / 2;
    }

    public double getX() {
        return pose.getX();
    }

    public double getY() {
        return pose.getY();
    }

    public Rotation2d getRotation() {
        return pose.getRotation();
    }

    public Pose2d getPos() {
        return pose;
    }
}