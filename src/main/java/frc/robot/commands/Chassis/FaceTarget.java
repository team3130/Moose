package frc.robot.commands.Chassis;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotMap;
import frc.robot.SupportingClassess.Chooser;
import frc.robot.sensors.vision.Limelight;
import frc.robot.subsystems.Chassis;

public class FaceTarget extends CommandBase {
    // defining an instance to be used throughout the command and to be instantiated in the constructor of type parameter
    private final Chassis m_chassis;
    private final Limelight m_limelight;

    private double angle = 0;

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
        m_chassis.configRampRate(RobotMap.kMaxRampRate);
        angle = m_chassis.getAngle() - m_limelight.getHeading().getDegrees();
        m_chassis.setSpinnySetPoint(angle);
        m_chassis.updatePIDValues();
    }

    @Override
    public void execute() {
        m_chassis.spinToAngle(m_chassis.getAngle());
    }

    @Override
    public boolean isFinished() {
        return Math.abs(angle - m_chassis.getAngle()) <= 2 || m_limelight.hasTrack();
    }

    @Override
    public void end(boolean interrupted) {
        m_chassis.configRampRate(0);
    }
}
