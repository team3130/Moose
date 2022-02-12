package frc.robot.team3130.SupportingClasses;

import junit.framework.TestCase;

import java.util.Arrays;

public class BallManagerTest extends TestCase {
    public void testGetPath() {
        Graph graph = new Graph();
        BallManager manager = new BallManager(graph);

        // max allowed value
        double max = 15;
        // min allowed value
        double min = -15;

        // test case #1
        Double[][][] toAdd = new Double[][][] {
                { { 0.0, 1.0 }, { 1.0, 2.0 }, { 2.0, 3.0 }, { 3.0, 3.0 }, { 4.0, 2.0 } },
                { { 0.05, 1.05 }, { 1.0, 2.0 }, { 2.0, 3.0 }, { 3.0, 3.0 }, { 4.0, 2.0 } },
                { { 0.1, 1.1 }, { 1.0, 2.0 }, { 2.0, 3.0 }, { 3.0, 3.0 }, { 4.0, 2.0 } },
                { { 0.1, 1.1 }, { 1.0, 2.0 }, { 2.0, 3.0 }, { 3.0, 3.0 }, { 4.0, 2.0 } },
                { { 0.1, 1.1 }, { 1.0, 2.0 }, { 2.0, 3.0 }, { 3.0, 3.0 }, { 4.0, 2.0 } } };

        for (Double[][] temp : toAdd) {
            for (Double[] coords : temp) {
                graph.addNode(new BlueBall(coords[0], coords[1]));
            }
        }
        for (int i = 0; i < graph.nodes.size(); i++) {
            System.out.println(i + ": " + graph.nodes.get(i));
        }
    }
}