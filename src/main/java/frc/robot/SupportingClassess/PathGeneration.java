package frc.robot.SupportingClassess;

import frc.robot.sensors.vision.WheelSpeedCalculations;

public class PathGeneration {
    protected final double[] range;

    public PathGeneration(WheelSpeedCalculations wheelSpeedCalculations) {
        range = new double[] {
                wheelSpeedCalculations.getMainDataStorage().get(1).getDistance(),
                wheelSpeedCalculations.getMainDataStorage().get(wheelSpeedCalculations.getMainDataStorage().size() - 2).getDistance()
        };
    }

    public void
}
