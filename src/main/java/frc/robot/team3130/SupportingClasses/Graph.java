package frc.robot.team3130.SupportingClasses;

import javax.swing.text.Position;
import java.util.ArrayList;

public class Graph {
    private int nodeCount = 25;

    ArrayList<Node> nodes = new ArrayList<>(nodeCount);
    ArrayList<ArrayList<Double>> distances = new ArrayList<>(nodeCount);

    public Graph() {
        for (int i = 0; i < nodes.size(); i++) {
            distances.add(new ArrayList<>(nodeCount));
        }
    }

    // Adds node to nodes ArrayList, then adds distances to graph
    public void addNode(Node node) {
        nodes.add(node);
        setDistances(nodes.indexOf(node));
    }

    // Fills a new line in the matrix with distances to each existing node
    private void setDistances(int index) {
        for (int j = 0; j < distances.size(); j++) {
            double distance = nodes.get(index).distance(nodes.get(j));
            if (index == j) {
                distances.get(index).set(j, distance);
            }
            else {
                distances.get(index).set(j, distance);
                distances.get(j).set(index, distance);
            }
        }
    }

    // idk why I have this I guess just to make sure the graph full
    public void setAllDistances() {
        for (int i = 0; i < nodes.size(); i++) {
            for (int j = 0; j < nodes.size(); j++) {
                distances.get(i).set(j, nodes.get(i).distance(nodes.get(j)));
            }
        }
    }

    /**
     * Removes a node from the nodes ArrayList and
     * removes corresponding distances from the graph
     */
    public void clearNode(Node node) {
        int index = nodes.indexOf(node);
        if (index != -1) {
            clearDistances(index);
            nodes.remove(index);
        } else {
            System.out.println("bruh wtf are you doing that's not even in the Graph");
        }
    }

    /**
     * Clears the matrix along the indeces [index]
     * 
     * @param index
     */
    public void clearDistances(int index) {
        for (int j = 0; j < distances.size(); j++) {
            distances.get(j).remove(index);
        }
        distances.remove(index);
    }

    // clears balls in FOV, then adds new balls
    public void refreshVisible(Node[] newNodes, CameraFOV bounds) {
        for (Node node : nodes) {
            if (bounds.isInView(node)) {
                clearNode(node);
            }
        }
        for (Node node : newNodes) {
            addNode(node);
        }
    }

    /**
     * generates a path to shoot balls
     * @param position current position of the bot
     * @param shoot list of shooting positions
     * @return the path in terms of nodes
     */
    public ArrayList<Node> shootingPath(Pose position, Pose[] shoot) {
        double shortest = Double.MAX_VALUE;
        ArrayList<Node> path = new ArrayList<>(3);
        if (nodes.size() == 1) {
            for (int k = 0; k < shoot.length; k++) {
                double distance = position.distance(nodes.get(0))
                        + nodes.get(0).distance(shoot[k]);
                if (shortest > distance) {
                    shortest = distance;
                    path.add(nodes.get(0));
                    path.add(shoot[k]);
                }
            }
        } else {
            for (int i = 0; i < nodes.size(); i++) {
                for (int j = 1; j < nodes.size(); j++) {
                    if (i != j) {
                        for (int k = 0; k < shoot.length; k++) {
                            double distance = position.distance(nodes.get(i))
                                    + distances.get(i).get(j)
                                    + nodes.get(j).distance(shoot[k]);
                            if (shortest > distance) {
                                shortest = distance;
                                path.add(nodes.get(i));
                                path.add(nodes.get(j));
                                path.add(shoot[k]);
                            }
                        }
                    }
                }
            }
        }
        return path;
    }

    public ArrayList<Node> defensePath(Pose position) {
        double shortest = Double.MAX_VALUE;
        ArrayList<Node> path = new ArrayList<>(2);

        if (nodes.size() == 1) {
            path.add(nodes.get(0));
        } else {
            for (int i = 0; i < distances.size(); i++) {
                for (int j = 1; j < distances.get(i).size(); j++) {
                    if (i != j) {
                        double distance = position.distance(nodes.get(i))
                                + distances.get(i).get(j);
                        if (shortest > distance) {
                            shortest = distance;
                            path.add(nodes.get(i));
                            path.add(nodes.get(j));
                        }
                    }
                }
            }
        }
        return path;
    }

}