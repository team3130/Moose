package frc.robot.team3130.SupportingClasses;

import edu.wpi.first.math.Pair;

public class BallManager {

    // Graph associated with this BallManager
    private final Graph graph;

    public BallManager(Graph graph) {
        this.graph = graph;
    }

    /**
     * add all the balls to their corresponding array in the Node class
     * @param balls from nanos
     */
    public void addBalls(Pair<Double, Double>[] balls) {
        for (Pair<Double, Double> ball : balls) {
            graph.addUnique(ball);
        }
    }

}
