package frc.robot.sensors.vision;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.utils.LinearInterp;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Comparator;

public class WheelSpeedCalculations {

    private static final Comparator<DataPoint> compPoint = Comparator.comparingDouble(p -> p.distance);

    private static class DataPoint {
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

    public WheelSpeedCalculations() {
        FILEPATH = Filesystem.getDeployDirectory() + File.separator + "RapidReactMarchFirst.csv";

        data_MainStorage = new ArrayList<>();
        readFile();
        speedCurve = null;
        loadCurve();
    }

    public void loadCurve() {
        ArrayList<Double> data_Dist = new ArrayList<>();
        ArrayList<Double> data_Speed = new ArrayList<>();

        for (int iii = 0; iii < data_MainStorage.size(); iii++) {
            DataPoint pt = data_MainStorage.get(iii);
            data_Dist.add(pt.distance);
            data_Speed.add(pt.speed);
        }

        speedCurve = new LinearInterp(data_Dist, data_Speed);
    }

    public void readFile() {
        data_MainStorage.clear();

        try (BufferedReader br = new BufferedReader(new FileReader(FILEPATH))) {
            for (String line; (line = br.readLine()) != null; ) {
                if (!line.equals("")) data_MainStorage.add(new DataPoint(line));
            }
            // line is not visible here.
        } catch (IOException e) {
            e.printStackTrace();
        }

        data_MainStorage.sort(compPoint);
        loadCurve();
    }

    public double getSpeed(Double dist) {
        return speedCurve.getY(dist);
    }

}

