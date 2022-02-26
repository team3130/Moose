package frc.robot.team3130.SupportingClasses;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Pose extends Node {
    private double rotation;

    public Pose(double xPos, double yPos, double rotation) {
        super(xPos, yPos);
        this.rotation = rotation;
    }

    public Pose(Pose2d pose) {
        this(pose.getX(), pose.getY(), pose.getRotation().getRadians());
    }

    public double getRotation() {
        return rotation;
    }

    public Pose2d toPose2d() {
        return new Pose2d(super.toTranslation2d(), new Rotation2d(rotation));
    }

}
