package frc.robot.team3130.SupportingClasses;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.team3130.Robot;
import frc.robot.team3130.RobotMap;
import frc.robot.team3130.subsystems.Chassis;

import java.util.ArrayList;
import java.util.Arrays;

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
     *  doesn't stall out
     *  Generate a trajectory based on balls
     * @return Trajectory for auton using cv
     */
    public Trajectory getPath() {
        TrajectoryConfig config = new TrajectoryConfig(RobotMap.kMaxVelocityMetersPerSec, RobotMap.kMaxAccelerationMetersPerSecondSq); //TODO: MAKE THE TRAJECTORY CONFIG

        Pose2d bot_pos = chassis.getPose();
        Node[] nodes = new Node[RobotMap.kShootingPoses.length];
        for (int i = 0; i < nodes.length; i++) {
            nodes[i] = new Node(RobotMap.kShootingPoses[i].getX(), RobotMap.kShootingPoses[i].getY(), RobotMap.kShootingPoses[i].getRotation());
        }

        ArrayList<Node> list = graph.shootingPath(new Node(bot_pos.getX(), bot_pos.getY(), bot_pos.getRotation()), nodes);

        ArrayList<Pose2d> waypoints = new ArrayList<>(list.size());

        list.forEach((Node node) -> waypoints.add(node.getPos()));

        return TrajectoryGenerator.generateTrajectory(waypoints, config);
    }

}
