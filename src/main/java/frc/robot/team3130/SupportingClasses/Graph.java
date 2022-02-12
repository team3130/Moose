package frc.robot.team3130.SupportingClasses;

import java.util.ArrayList;

import javax.naming.spi.DirStateFactory;

public class Graph {
    private int nodeCount = 25;

    ArrayList<Node> nodes = new ArrayList<Node>(nodeCount);
    ArrayList<ArrayList<Double>> distances = new ArrayList<ArrayList<Double>>(nodeCount);

    public Graph() {
        for (int i = 0; i < nodes.size(); i++)
            distances.add(new ArrayList<Double>(nodeCount));
    }

    public void addNode(Node node) {
        nodes.add(node);
        setDistances(nodes.indexOf(node));
    }

    public void setDistances(int index) {
        for (int j = 0; j < distances.size(); j++) {
            double distance = nodes.get(index).distance(nodes.get(j));
            if (index == j)
                distances.get(index).set(j, distance);
            else {
                distances.get(index).set(j, distance);
                distances.get(j).set(index, distance);
            }
        }
    }

    public void setAllDistances() {
        for (int i = 0; i < nodes.size(); i++) {
            for (int j = 0; j < nodes.size(); j++) {
                distances.get(i).set(j, nodes.get(i).distance(nodes.get(j)));
            }
        }
    }

    public void clearNode(Node node) {
        int index = nodes.indexOf(node);
        if (index != -1) {
            clearDistances(index);
            nodes.remove(index);
        }
        else System.out.println("bruh wtf are you doing that's not even in the Graph");
    }

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

        }
    }

}