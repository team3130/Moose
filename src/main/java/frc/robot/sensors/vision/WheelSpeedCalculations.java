package frc.robot.sensors.vision;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.RobotMap;
import frc.robot.utils.LinearInterp;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import static frc.robot.sensors.vision.WheelSpeedCalculations.CurveMechanism.HOOD_WINCH;
import static frc.robot.sensors.vision.WheelSpeedCalculations.CurveMechanism.SHOOTER;

public class WheelSpeedCalculations {

    private static final Comparator<DataPoint> compPoint = Comparator.comparingDouble(p -> p.distance);

    private class DataPoint {
        double distance;
        double speed;

        public DataPoint(double dist, double speed) {
            distance = dist;
            this.speed = speed;
        }

        public DataPoint(String point) {
            if (point.contains(",")) {
                String[] parts = point.split(",");
                distance = Double.parseDouble(parts[0]);
                speed = Double.parseDouble(parts[1]);
            }
        }

        @Override
        public String toString() {
            return "(" + distance + "," + speed + ")";
        }

        @SuppressWarnings("unused")
        public boolean equals(DataPoint other) {
            return distance == other.distance;
        }
    }


    private ArrayList<DataPoint> data_MainStorage;
    private LinearInterp speedCurve;
    private final String FILEPATH;

    public static enum CurveMechanism {SHOOTER, HOOD_WINCH}
    private CurveMechanism mechanism;

    public WheelSpeedCalculations(CurveMechanism mechanism) {
        this.mechanism = mechanism;
        if (mechanism == SHOOTER) {
            FILEPATH = Filesystem.getDeployDirectory() + File.separator + "curves" + File.separator + "ShooterPlaceHolder.csv";
        } else if (mechanism == HOOD_WINCH) {
            FILEPATH = Filesystem.getDeployDirectory() + File.separator + "curves" + File.separator + "HoodPlaceHolder.csv";
        } else {
            FILEPATH = null;
        }


        data_MainStorage = new ArrayList<>();
        readFile();
        speedCurve = null;
        loadCurve();
    }

    public void loadCurve() {
        ArrayList<Double> data_Dist = new ArrayList<>();
        ArrayList<Double> data_Speed = new ArrayList<>();

        for (DataPoint pt : data_MainStorage) {
            data_Dist.add(pt.distance);
            data_Speed.add(pt.speed);
        }

        speedCurve = new LinearInterp(data_Dist, data_Speed);

}
    public void readFile() {
        data_MainStorage.clear();

        try (BufferedReader br = new BufferedReader(new FileReader(FILEPATH))) {
            for (String line; (line = br.readLine()) != null; ) {
                if (!line.equals("")) {
                    data_MainStorage.add(new DataPoint(line));
                }
            }
        }
        catch (IOException e) {
            data_MainStorage.addAll(List.of(
                    new DataPoint(0, (mechanism == SHOOTER) ? 3200 : 0),
                    new DataPoint(500, (mechanism == SHOOTER) ? 3200 : 0)));
            DriverStation.reportError("There was a problem in Wheel Speed Calculations", RobotMap.debug);
        }

        data_MainStorage.sort(compPoint);
        loadCurve();
    }

    public double getSpeed(Double dist) {
        return speedCurve.getY(dist);
    }

}

