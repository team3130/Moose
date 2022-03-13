package frc.robot.SupportingClassess;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonCommand {

    protected Pose2d position;
    protected CommandBase cmd;

    public AutonCommand(CommandBase cmd, Pose2d position) {
        this.cmd = cmd;
        this.position = position;
    }

    public void setPosition(Pose2d newPosition) {
        position = newPosition;
    }

    public void setPosition(double x, double y) {
        setPosition(x, y, 0);
    }

    public void setPosition(double x, double y, double rad) {
        setPosition(x, y, new Rotation2d(rad));
    }

    public void setPosition(double x, double y, Rotation2d rotation) {
        position = new Pose2d(x, y, rotation);
    }

    public Pose2d getPosition() {
        return position;
    }

    public CommandBase getCmd() {
        return cmd;
    }
}
