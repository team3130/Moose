package frc.robot.team3130.SupportingClasses;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;

public class GraphPath implements Comparable<GraphPath>{
    private double distance;
    private final Node[] path;
    private int newIndex;

    public GraphPath(double distance, Node[] path) {
        this.distance = distance;
        this.path = path.clone();
        newIndex = getNewestIndex();
    }

    public int getNewestIndex() {
        for (int i = 0; i < path.length; i++) {
            if (path[i] == null) {
                return i;
            }
        }
        return -1;
    }

    public Node getLast() {
        return path[newIndex];
    }

    public Node getSecondLast() {
        return path[newIndex - 1];
    }

    public GraphPath(double distance, int items) {
        this(distance, new Node[items]);
    }

    public double getDistance() {
        return distance;
    }

    public void setDistance(double distance) {
        this.distance = distance;
    }

    public void addDistance(double distance) {
        this.distance += distance;
    }

    public Node[] getPath() {
        return path;
    }

    public void addNodeToPath(Node nextNode) {
        path[newIndex++] = nextNode;
    }

    public void addNodeToPath(Node nextNode, double distance) {
        addNodeToPath(nextNode);
        addDistance(distance);
    }

    public int getSteps() {
        return newIndex;
    }

    public GraphPath copy() {
        return new GraphPath(distance, path.clone());
    }

    public String toString() {
        return "path: " + Arrays.toString(path);
    }

    @Override
    public int compareTo(GraphPath other) {
        return Double.compare(distance, other.distance);
    }
}
