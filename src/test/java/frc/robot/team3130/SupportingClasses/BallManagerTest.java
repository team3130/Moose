package frc.robot.team3130.SupportingClasses;

import edu.wpi.first.math.Pair;
import junit.framework.TestCase;

import java.util.Arrays;

public class BallManagerTest extends TestCase {
    public void testGetPath() {
        Graph graph = new Graph();
        BallManager manager = new BallManager(graph);

        // first layer is time, second is batch, third is just of size two and is the x y coords
        double[][][] toAdd = new double[10][5][2];

        // max allowed value
        double max = 15;
        // min allowed value
        double min = -15;

        for (int time = 0; time < toAdd.length; time++) {
            for (int batch = 0; batch < toAdd[time].length; batch++) {
                for (int coords = 0; coords < toAdd[time][batch].length; coords++) {
                    Pair<Double, Double> add = new Pair<>((Math.random() * (max - min)) + min, (Math.random() * (max - min)) + min);
                    graph.addUnique(add);
                    System.out.print("(" + Math.round(add.getFirst()*100)/100 + ", " + Math.round(add.getSecond()*100)/100 + "), ");
//                    toAdd[time][batch][coords] = add.getFirst();
//                    toAdd[time][batch][coords] = add.getSecond();
                }
                System.out.println();
            }
            System.out.println();
        }
        System.out.println("\n\n\n" + graph);
    }
}