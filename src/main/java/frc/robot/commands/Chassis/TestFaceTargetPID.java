package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.subsystems.Chassis;

public class TestFaceTargetPID extends CommandBase {
    // defining an instance to be used throughout the command and to be instantiated in the constructor of type parameter
    private final Chassis m_chassis;

    private double angle = 0;

    public TestFaceTargetPID(Chassis chassis) {
        //mapping to object passed through parameter
        m_chassis = chassis;
        m_requirements.add(chassis);
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        m_chassis.configRampRate(RobotMap.kMaxRampRate);
        m_chassis.updatePIDValues();
        angle = m_chassis.getAngle() + 45;
        m_chassis.setSpinnySetPoint(angle);
        m_chassis.resetPIDLoop();
    }

    @Override
    public void execute() {
        m_chassis.faceTarget(m_chassis.getAngle());
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
