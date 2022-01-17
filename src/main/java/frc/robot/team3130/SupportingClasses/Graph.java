package frc.robot.team3130.SupportingClasses;

import edu.wpi.first.math.Pair;

import java.util.*;

public class Graph {
    // hashmap to cache the positions in the array that each node is at
    HashMap<Node, Integer> nodeMap;
    public final ArrayList<Node> nodes;

    // adjacency matrix
    private double[][] adjMatrix;

    /**
     * The only constructor of graph to be called once on robotInit
     */
    public Graph() {
        nodes = new ArrayList<>();
        nodeMap = new HashMap<>();
        adjMatrix = new double[3][3];

        for (int j = 0; j < adjMatrix.length; j++) {
            for (int k = 0; k < adjMatrix.length; k++) {
                adjMatrix[j][k] = 0;
            }
        }

        nodes.add(new Node(1, 1));
        nodeMap.put(nodes.get(0), 0);
    }

    /**
     * To be called by connectNode in a for loop
     * @param toBeAdded node that will be added (the one added most recently)
     * @param ConnectedTo node that it will be connected to
     */
    private void putNodeInGraph(Node toBeAdded, Node ConnectedTo) {
        double distance = toBeAdded.getDistance(ConnectedTo);
        // add to directed graph
        adjMatrix[nodeMap.get(toBeAdded)][nodeMap.get(ConnectedTo)] = distance;
        adjMatrix[nodeMap.get(ConnectedTo)][nodeMap.get(toBeAdded)] = distance;
    }

    /**
     * Designed to connect nodes in the graph that involve the bot
     * instead of using getAngleToForm we instead use the
     * @param botNode the node that corresponds to nodes.get(0) aka the bot
     * @param ConnectedTo the node connecting to
     */
    private void putNodeInGraphConnectedToBot(Node botNode, Node ConnectedTo) {
        double botAngle = 0;
        // distance
        double distance = botNode.getDistance(ConnectedTo);
        // add to undirected graph
        adjMatrix[nodeMap.get(botNode)][nodeMap.get(ConnectedTo)] = distance;
        adjMatrix[nodeMap.get(ConnectedTo)][nodeMap.get(botNode)] = distance;
    }

    /**
     * add newNode to the arrayList, put it, and it's index in a map, resize the matrix, and connect the nodes
     * @param newNode node that will be added to the graph
     */
    public void addNode(Node newNode) {
        if (!contains(newNode)) {
            nodes.add(newNode);
            nodeMap.put(newNode, nodes.size() - 1);
        }
        resize();
        ConnectNode(newNode);
    }

    /**
     * checks if the node exists
     * @param comparator the node that is being checked
     * @return if the node is registered
     */
    public boolean contains(Node comparator) {
        return nodeMap.containsKey(comparator);
    }

    /**
     * Used in testing only
     * @return whether it contains duplicates
     */
    public boolean containsDuplicates() {
        Set<Node> seen = new HashSet<>();
        for (Node node : nodes) {
            if (seen.contains(node)) {
                return true;
            }
            else {
                seen.add(node);
            }
        }
        return false;
    }

    public void addUnique(Double[] toBeAdded) {
        int numAppliesTo = 0;
        int[] indexApartOf = new int[nodes.size()];
        for (int i = 0; i < nodes.size(); i++) {
            if (nodes.get(i).canBeAPart(toBeAdded)) {
                indexApartOf[numAppliesTo++] = i;
            }
        }

        Node toAdd = new Node(toBeAdded[0], toBeAdded[1]);

        if (numAppliesTo == 0) {
            addNode(toAdd);
        }

        else if (numAppliesTo == 1) {
             nodes.get(indexApartOf[0]).addPair(toBeAdded);
        }

        // add possible balls to another Graph
        else {
            double shortest_distance = Double.MAX_VALUE;
            int index_of_short = -1;
            for (int i = 0; i < numAppliesTo; i++) {
                Node temp = new Node(nodes.get(i).getX_pos_Recent(), nodes.get(i).getY_pos_Recent());

                double distance = toAdd.getDistance(temp);

                if (distance < shortest_distance) {
                    index_of_short = i;
                }
            }
            nodes.get(index_of_short).addPair(toBeAdded);
        }
    }

    /**
     * Connects the node to every node in the graph
     * @param newNode the node to be connected
     */
    private void ConnectNode(Node newNode) {
        for (int i = 0; i < nodes.size(); i++) {
            // checks to make sure it doesn't add itself
            // we wouldn't need this if we just didn't iterate to the last element however threading issues could occur
            if (nodes.get(i) != newNode && i != 0) {
                putNodeInGraph(newNode, nodes.get(i));
            }
            // will run if new Node is not current and i == 0;
            else if (nodes.get(i) != newNode && i == 0) {
                // connect off of distance
                putNodeInGraphConnectedToBot(nodes.get(i), newNode);
            }
        }
    }

    private void resize() {
        double[][] newMatrix = new double[nodes.size()][nodes.size()];
            for (int j = 0; j < adjMatrix.length; j++) {
                for (int k = 0; k < adjMatrix.length; k++){
                    // check if the index is in bounds
                    if (j < newMatrix.length && k < newMatrix.length) {
                        newMatrix[j][k] = adjMatrix[j][k];
                }
            }
        }
        adjMatrix = newMatrix;
    }

    /**
     * Initial call for permutation
     * @return the shortest path
     */
    public GraphPath permute() {
        // nodes without the first node because that is the bot
        Node[] nodeses = new Node[nodes.size()];
        // for looper to add every item except for the 0th index in nodes
        for (int looper = 1; looper < nodes.size(); looper++) {
            nodeses[looper - 1] = nodes.get(looper);
        } 
        // "Array" of the shortest Graph paths should always be of size one
        GraphPath[] shortest = new GraphPath[] {new GraphPath(Double.MAX_VALUE, adjMatrix.length)};
        // initial call to the recursive function
        Permutation(0, nodeses, shortest);
        // because of the array we don't need to worry about the method returning anything
        return shortest[0];
    }

    /**
     * Recursive method to permute through all possible ones
     * @param index1 index used in method, pass in 0 for initial call
     * @param nodes the array of the sample nodes
     * @param shortest an array of size one to pass the shortest array by reference, because java sucks
     */
    private void Permutation(int index1, Node[] nodes, GraphPath[] shortest) {
        // base case that checks if we have iterated to the end of the list
        if (index1 != nodes.length - 1) {
            // for loop
            for (int index2 = index1, l = nodes.length; index2 < l; index2++) {
                // swap two indices to permute through the array using heaps algorithm
                swap(nodes, index1, index2);
                // store the distance in a double variable for comparison
                double tempDistance = determineDistance(nodes);
                // check if the distance is less than the shortest one seen so far
                if (tempDistance < shortest[0].getDistance()) {
                    // temporary graph path that reflects the current node
                    GraphPath temp = new GraphPath(0, adjMatrix.length);
                    // iterate to add items to the temporary GraphPath
                    for (Node node : nodes)  temp.addNodeToPath(node);
                    // determine the distance that this path takes
                    temp.setDistance(tempDistance);
                    shortest[0] = temp;
                }
                // re-call the method here
                Permutation(index1 + 1, nodes, shortest);
                // un-swap the array at the end of each iteration of the for looper
                swap(nodes, index1, index2);
            }
        }
    }

    /**
     * Determines the "distance" traveled
     * Summation of the weights
     * @param arr the path combination
     * @return the distance traveled
     */
    private double determineDistance(Node[] arr) {
        // the first one is going to the first node and the second one is getting to the second node
        // remember if all of them are wrong by a similar amount then it doesn't matter too much I hope
        double distanceTemp = adjMatrix[0][nodeMap.get(arr[0])] + adjMatrix[nodeMap.get(arr[0])][nodeMap.get(arr[1])];
        // iterate through array except for the first and last item
        for (int looper = 2; looper < arr.length; looper++) {
            // add distance of getting to the looper node from using the past two
            distanceTemp += adjMatrix[nodeMap.get(arr[looper - 1])][nodeMap.get(arr[looper])];
        }
        // return the distance
        return distanceTemp;
    }

    /**
     * Swaps items in a given Node Array
     * @param arr array that the values get swapped in
     * @param index1 the index of the first element to be swapped
     * @param index2 the index of the second element to be swapped
     */
    private void swap(Node[] arr, int index1, int index2) {
        Node temp = arr[index1];
        arr[index1] = arr[index2];
        arr[index2] = temp;
    }

    public String toString() {
        return nodes.toString();
    }

}
