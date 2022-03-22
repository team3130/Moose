package frc.robot.SupportingClassess;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonCommand {

    protected Pose2d startPosition;

    public Pose2d getEndPosition() {
        return endPosition;
    }

    public void setEndPosition(Pose2d endPosition) {
        this.endPosition = endPosition;
    }

    protected Pose2d endPosition;
    protected CommandBase cmd;

    public AutonCommand(CommandBase cmd, Pose2d startPosition) {
        this.cmd = cmd;
        this.startPosition = startPosition;
    }

    public AutonCommand(CommandBase cmd, Pose2d startPosition, Pose2d endPosition) {
        this.cmd = cmd;
        this.startPosition = startPosition;
        this.endPosition = endPosition;
    }

    public void setStartPosition(Pose2d newPosition) {
        startPosition = newPosition;
    }

    public void setPosition(double x, double y) {
        setPosition(x, y, 0);
    }

    public void setPosition(double x, double y, double rad) {
        setPosition(x, y, new Rotation2d(rad));
    }

    public void setPosition(double x, double y, Rotation2d rotation) {
        startPosition = new Pose2d(x, y, rotation);
    }

    public Pose2d getStartPosition() {
        return startPosition;
    }

    public CommandBase getCmd() {
        return cmd;
    }
}
