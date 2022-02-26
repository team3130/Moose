package frc.robot.team3130.SupportingClasses;

import java.util.ArrayList;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.team3130.Robot;
import frc.robot.team3130.RobotMap;
import frc.robot.team3130.subsystems.Chassis;

public class BallManager {

    // Graph associated with this BallManager
    private final Graph graph;
    private final Chassis chassis;

    public BallManager(Graph graph, Chassis chassis) {
        this.graph = graph;
        this.chassis = chassis;
    }

    /**
     * add all the balls to their corresponding array in the Node class
     * 
     * @param balls from nanos
     */
    public void addBalls(Pair<Double, Double>[] balls) {
        for (Pair<Double, Double> ball : balls) {
            graph.addNode(new Node(ball.getFirst(), ball.getSecond()));
        }
    }

    /**
     * Should be protected with its own thread so that the bot
     * doesn't stall out
     * Generate a trajectory based on balls
     * 
     * @return Trajectory for auton using cv
     */
    public Trajectory getPath(TrajectoryConfig config) {
        Pose2d bot_pos = chassis.getPose();
        ArrayList<Translation2d> waypoints = new ArrayList<>(2);
        Pose2d end_pos;

        ArrayList<Node> path = graph.shootingPath(new Pose(bot_pos), RobotMap.kShootingPoses);

        for (int i = 0; i < path.size() - 1; i++) {
            waypoints.add(path.get(i).toTranslation2d());
        }
        end_pos = ((Pose) path.get(path.size() - 1)).toPose2d();

        return TrajectoryGenerator.generateTrajectory(bot_pos, waypoints, end_pos, config);
    }

}
