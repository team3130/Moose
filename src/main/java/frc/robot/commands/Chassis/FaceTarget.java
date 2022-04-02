package frc.robot.commands.Chassis;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.vision.Limelight;
import frc.robot.subsystems.Chassis;

public class FaceTarget extends CommandBase {
    // defining an instance to be used throughout the command and to be instantiated in the constructor of type parameter
    private final Chassis m_chassis;
    private final Limelight m_limelight;

    private double angle = 0;

    private double optional = 0;

    public FaceTarget(Chassis chassis, Limelight limelight) {
        //mapping to object passed through parameter
        m_chassis = chassis;
        m_requirements.add(chassis);
        m_limelight = limelight;
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        m_chassis.configRampRate(2);
        m_chassis.updatePIDValues();
        m_chassis.resetOdometry(new Pose2d());
        angle = m_chassis.getAngle() - m_limelight.getHeading().getDegrees();
        m_chassis.setSpinnySetPoint(angle);
        m_chassis.setLateralSetPoint(0);
        m_chassis.resetPIDLoop();
    }

    @Override
    public void execute() {
        m_chassis.faceTarget(m_chassis.getAngle(), m_chassis.getCurrentVectorDist());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_chassis.configRampRate(0);
    }
}
