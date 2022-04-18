package frc.robot.SupportingClassess;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class Event {
    protected Pose2d poseGoingTo;
    protected EventType type;
    protected Command command;

    public Event(Pose2d poseGoingTo, Command toRun, EventType type) {
        this.poseGoingTo = poseGoingTo;
        this.type = type;
        this.command = toRun;
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

    public Command getCmd() {
        return command;
    }

    public void setCommand(Command replacement) {
        command = replacement;
    }
}
