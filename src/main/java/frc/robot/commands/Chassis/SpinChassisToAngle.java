package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.subsystems.Chassis;

public class SpinChassisToAngle extends CommandBase {
    // defining an instance to be used throughout the command and to be instantiated in the constructor of type parameter
    private final Chassis m_chassis;
    private double angle;
    private Timer timer;
    private double time = 1; //TODO at GF needs tuning

    public SpinChassisToAngle(Chassis chassis, double angle) {
        //mapping to object passed through parameter
        m_chassis = chassis;
        m_requirements.add(chassis);
        this.angle = angle;
        timer = new Timer();
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        m_chassis.configRampRate(RobotMap.kMaxRampRate);
        m_chassis.updatePIDValues();
        m_chassis.setSpinnySetPoint(angle + m_chassis.getAngle());
        m_chassis.resetPIDLoop();
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        m_chassis.spinToAngle(m_chassis.getAngle());
    }

    @Override
    public boolean isFinished() {
        return m_chassis.getAtSetpoint() || timer.hasElapsed(time);
    }

    @Override
    public void end(boolean interrupted) {
        m_chassis.configRampRate(0);
        timer.stop();
    }
}
