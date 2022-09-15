package frc.robot.sensors.vision;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.SupportingClassess.GeneralUtils;
import frc.robot.utils.LinearInterp;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Map;

import static frc.robot.sensors.vision.WheelSpeedCalculations.CurveMechanism.SHOOTER;

public class WheelSpeedCalculations implements GeneralUtils {

    private static final Comparator<DataPoint> compPoint = Comparator.comparingDouble(p -> p.distance);

    private ShuffleboardTab tab = Shuffleboard.getTab("Wheel Speed Calculations");

    private NetworkTableEntry sliderOffsetRPM = tab
            .add("Slider Offset", 0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -5, "max", 5))
            .getEntry();
    private int CurrentOffset = 0;

    @Override
    public void outputToShuffleboard() {
        if (sliderOffsetRPM.getDouble(CurrentOffset) != CurrentOffset) {
            BallPressureChange((int) (sliderOffsetRPM.getDouble(CurrentOffset) - CurrentOffset));
            CurrentOffset = (int) sliderOffsetRPM.getDouble(CurrentOffset);
        }
    }

    public void ModifySlider(boolean HRL) {
        if (HRL) {
            sliderOffsetRPM.setNumber(sliderOffsetRPM.getDouble(CurrentOffset) + 1);
        }
        else {
            sliderOffsetRPM.setNumber(sliderOffsetRPM.getDouble(CurrentOffset) - 1);
        }
    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void disable() {

    }

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

        public void AddSpeed(double Speeeeeed) {
            speed += Speeeeeed;
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
    private String FILEPATH;

    public enum CurveMechanism {SHOOTER, HOOD_WINCH}
    private CurveMechanism mechanism;

    public WheelSpeedCalculations(CurveMechanism mechanism) {
        this.mechanism = mechanism;
        FILEPATH = Filesystem.getDeployDirectory() + File.separator + "curves";


        FILEPATH = FILEPATH + File.separator + "Marriuci.csv";



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
            DriverStation.reportError("Could not read CSV, defaulting", false);
        }

        data_MainStorage.sort(compPoint);
        loadCurve();
    }

    public void BallPressureChange (int PressureScale) {
        for (int i = 0; i < data_MainStorage.size(); i++) {
            data_MainStorage.get(i).AddSpeed(50 * PressureScale);
        }
        loadCurve();
    }

    public double getSpeed(Double dist) {
        return speedCurve.getY(dist);
    }

}

