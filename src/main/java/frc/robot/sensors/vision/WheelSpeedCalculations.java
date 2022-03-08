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
        double angle;

        public DataPoint(double dist, double speed, double angle) {
            distance = dist;
            this.speed = speed;
            this.angle = angle;
        }

        public DataPoint(String point) {
            if (point.contains(",")) {
                String[] parts = point.split(",");
                distance = Double.parseDouble(parts[0]);
                speed = Double.parseDouble(parts[1]);
                angle = Double.parseDouble(parts[2]);
            }
        }

        @Override
        public String toString() {
            return "(" + distance + "," + speed + "," + angle + ")";
        }

        @SuppressWarnings("unused")
        public boolean equals(DataPoint other) {
            return distance == other.distance;
        }
    }


    private ArrayList<DataPoint> data_MainStorage;
    private LinearInterp speedCurve;
    private LinearInterp angleCurve;
    private final String FILEPATH;

    public WheelSpeedCalculations() {
        FILEPATH = Filesystem.getDeployDirectory() + File.separator + "curves" + File.separator + "RapidReactMarchFirst.csv";

        data_MainStorage = new ArrayList<>();
        readFile();
        speedCurve = null;
        angleCurve = null;
        loadCurve();
    }

    public void loadCurve() {
        ArrayList<Double> data_Dist = new ArrayList<>();
        ArrayList<Double> data_Speed = new ArrayList<>();
        ArrayList<Double> data_Angle = new ArrayList<>();

        for (int iii = 0; iii < data_MainStorage.size(); iii++) {
            DataPoint pt = data_MainStorage.get(iii);
            data_Dist.add(pt.distance);
            data_Speed.add(pt.speed);
            data_Angle.add(pt.angle);
        }

        speedCurve = new LinearInterp(data_Dist, data_Speed);
        angleCurve = new LinearInterp(data_Dist, data_Angle);
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
    public double getAngle(Double dist){return angleCurve.getY(dist);}

}

