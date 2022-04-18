package frc.robot.SupportingClassess;

import edu.wpi.first.math.geometry.Pose2d;

public class Event {
    protected Pose2d poseGoingTo;
    protected EventType type;

    public Event(Pose2d poseGoingTo, EventType type) {
        this.poseGoingTo = poseGoingTo;
        this.type = type;
    }

    public Pose2d getPoseGoingTo() {
        return poseGoingTo;
    }

    public void setPoseGoingTo(Pose2d poseGoingTo) {
        this.poseGoingTo = poseGoingTo;
    }

    public EventType getType() {
        return type;
    }

    public void setType(EventType type) {
        this.type = type;
    }
}
