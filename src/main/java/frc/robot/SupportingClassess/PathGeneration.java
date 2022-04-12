package frc.robot.SupportingClassess;

import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.sensors.vision.WheelSpeedCalculations;
import frc.robot.subsystems.Chassis;

public class PathGeneration {
    protected final double[] range;

    protected final Chassis m_chassis;


    public PathGeneration(WheelSpeedCalculations wheelSpeedCalculations, Chassis chassis) {
        range = new double[] {
                wheelSpeedCalculations.getMainDataStorage().get(1).getDistance(),
                wheelSpeedCalculations.getMainDataStorage().get(wheelSpeedCalculations.getMainDataStorage().size() - 2).getDistance()
        };

        m_chassis = chassis;
    }

    public Trajectory goToShoot() {

    }
}
