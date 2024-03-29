package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.subsystems.Chassis;

public class SpinChassisToAngle extends CommandBase {
    // defining an instance to be used throughout the command and to be instantiated in the constructor of type parameter
    private final Chassis m_chassis;
    private double angle;

    private Timer timer = new Timer();

    public SpinChassisToAngle(Chassis chassis, double angle) {
        //mapping to object passed through parameter
        m_chassis = chassis;
        m_requirements.add(chassis);
        this.angle = angle;
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        m_chassis.configRampRate(RobotMap.kMaxRampRate);
        m_chassis.updatePIDValues();
        m_chassis.resetPIDLoop();
        m_chassis.setSpinnySetPoint(angle + m_chassis.getSpinnyAngle());

        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        m_chassis.spinOutput();
    }

    @Override
    public boolean isFinished() {
        return m_chassis.getAtSetpoint() || timer.hasElapsed(1);
    }

    @Override
    public void end(boolean interrupted) {
        m_chassis.configRampRate(0);
        timer.stop();
    }
}
