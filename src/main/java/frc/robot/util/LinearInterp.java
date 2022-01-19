package frc.robot.util;

import java.util.ArrayList;

public class LinearInterp {

    private ArrayList<Double> dist;
    private ArrayList<Double> speed;

    boolean sane = true;

    public LinearInterp(ArrayList<Double> dist, ArrayList<Double> speed) {
        this.dist = dist;
        this.speed = speed;

        if (!isInputSane())
            sane = false;
        return;
    }

    public boolean isSane() {
        return sane;
    }

    /**
     * Returns the y value corresponding to the Linear Interpolation of the data provided
     *
     * @param x the x value to find the corresponding y of
     * @return the y value that matches x
     */
    public double getY(double x) {
        int distSize = dist.size() - 1;
        int interval_begin = 0;
//        System.out.println(distSize);
        for(int i = 0; i < distSize; i++) {
            if(dist.get(i) <= x && x < dist.get(i + 1)){
                interval_begin = i;
                break;
            }
        }
        double m = ((speed.get(interval_begin + 1) - speed.get(interval_begin)) / (dist.get(interval_begin + 1) - dist.get(interval_begin)));

        return m * (x - dist.get(interval_begin)) + speed.get(interval_begin);
    }

    /**
     * Checks if the input is in a usable state for interpolation
     * <p>Returns false if either array is empty, if they aren't equal to eachother, or if the distance array isn't sorted
     *
     * @return if the Input arrays are sane
     */
    private boolean isInputSane() {
        if (dist.size() == 0 || speed.size() == 0) {
            System.out.println("Empty Array");
            return false;
        }
        if (dist.size() != speed.size()) {
            System.out.println(dist.size());
            System.out.println(speed.size());
            System.out.println("Diff Sizes");
            return false;
        }

        return isSorted(dist);
    }

    /**
     * Checks if an arrayList is sorted
     * <p>Returns false if the array list has a higher number followed by a lower one, or if there are duplicate numbers in a row
     *
     * @param list the array list to check
     * @return if the list is sorted
     */
    private boolean isSorted(ArrayList<Double> list) {
        for (int iii = 0; iii < list.size() - 1; iii++) {
            if (list.get(iii) >= list.get(iii + 1))
                return false;
        }
        return true;
    }
}
