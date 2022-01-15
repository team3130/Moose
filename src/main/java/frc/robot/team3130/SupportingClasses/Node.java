package frc.robot.team3130.SupportingClasses;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;

public class Node {
    protected final ArrayList<Double[]> poses;

    public Node(double posX, double posY) {
        this.poses = new ArrayList<>();
        this.poses.add(new Double[]{posX, posY});
    }
    public double getX_pos_Recent() {
        return poses.get(this.poses.size() - 1)[0];
    }

    public double getY_pos_Recent() {
        return poses.get(this.poses.size() - 1)[1];
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Node node = (Node) o;
        // if its within 0.1 units, it's the same ball
        return this.poses.get(this.poses.size() - 1)[0] < node.poses.get(this.poses.size() - 1)[0] + 0.1 && this.poses.get(this.poses.size() - 1)[0] > node.poses.get(this.poses.size() - 1)[0] - 0.1 && this.poses.get(this.poses.size() - 1)[1] < node.poses.get(this.poses.size() - 1)[1] + 0.1 && this.poses.get(this.poses.size() - 1)[1] > node.poses.get(this.poses.size() - 1)[1] - 0.1;
    }

    public double getRelAngle(Node otherNode) {
        int offset = 0;
        int sign = 1;
        if (otherNode.poses.get(this.poses.size() - 1)[0] - this.poses.get(this.poses.size() - 1)[0] < 0 && otherNode.poses.get(this.poses.size() - 1)[1] - this.poses.get(this.poses.size() - 1)[1] > 0) {
            offset = 90;
        }

        else if (otherNode.poses.get(this.poses.size() - 1)[0] - this.poses.get(this.poses.size() - 1)[0] < 0 && otherNode.poses.get(this.poses.size() - 1)[1] - this.poses.get(this.poses.size() - 1)[1] < 0) {
            offset = 90;
            sign = -1;
        }

        else if (otherNode.poses.get(this.poses.size() - 1)[0] - this.poses.get(this.poses.size() - 1)[0] > 0 && otherNode.poses.get(this.poses.size() - 1)[1] - this.poses.get(this.poses.size() - 1)[1] < 0) {
            sign = -1;
        }

        double degrees = sign * (Math.toDegrees(Math.acos(Math.abs((otherNode.poses.get(this.poses.size() - 1)[0] - this.poses.get(this.poses.size() - 1)[0]) / getDistance(otherNode)))) + offset);

        if (Double.isNaN(degrees)) {
            System.out.println("\n\nArc cosine: " + Math.acos(Math.abs((otherNode.poses.get(this.poses.size() - 1)[0] - this.poses.get(this.poses.size() - 1)[0]) / getDistance(otherNode))));
            System.out.println("Passed into arc cosine: " + Math.abs((otherNode.poses.get(this.poses.size() - 1)[0] - this.poses.get(this.poses.size() - 1)[0]) / getDistance(otherNode)));
            System.out.println("x - x: " + (otherNode.poses.get(this.poses.size() - 1)[0] - this.poses.get(this.poses.size() - 1)[0]));
            System.out.println("Distance: " + getDistance(otherNode));
            System.out.println("offset: " + offset);
            System.out.println("sign: " + sign);
            System.out.println("\n");
            return 0;
        }
        return degrees;
    }

    public void addPair(Pair<Double, Double> coords) {
        poses.add(new Double[] {coords.getFirst(), coords.getSecond()});
    }

    /**
     * Using the law of cosines this method gets the angle to one node if approached from another
     * @param to node going to
     * @param from node coming to this node from
     * @return the angle measure
     */
    public double getAngleToFrom(Node to, Node from) {
        // Side a in a triangle
        double A = from.getDistance(this);
        // Side b in a triangle
        double B = this.getDistance(to);
        // Side c in a triangle
        double C = to.getDistance(from);

        // check if A or B or C is 0 which would result in NaN
        if (A == 0 || B == 0 || C == 0) {
            return 0;
        }

        // set the heading using law of cosines
        double degrees = 180 - Math.toDegrees(Math.acos((Math.pow(A, 2) + Math.pow(B, 2) - Math.pow(C, 2)) / (2 * A * B)));

        // logic for different quadrants
        int offset = 0;
        int sign = 1;
        if (to.poses.get(this.poses.size() - 1)[0] - this.poses.get(this.poses.size() - 1)[0] < 0 && to.poses.get(this.poses.size() - 1)[1] - this.poses.get(this.poses.size() - 1)[1] > 0) {
            offset = 90;
        }

        else if (to.poses.get(this.poses.size() - 1)[0] - this.poses.get(this.poses.size() - 1)[0] < 0 && to.poses.get(this.poses.size() - 1)[1] - this.poses.get(this.poses.size() - 1)[1] < 0) {
            offset = 90;
            sign = -1;
        }

        else if (to.poses.get(this.poses.size() - 1)[0] - this.poses.get(this.poses.size() - 1)[0] > 0 && to.poses.get(this.poses.size() - 1)[1] - this.poses.get(this.poses.size() - 1)[1] < 0) {
            sign = -1;
        }

        // applying offsets
        degrees = sign * (degrees + offset);

        // debugging NaN
        if (Double.isNaN(degrees)) {
            System.out.println("\n\nArc cosine: " + Math.acos((Math.pow(A, 2) + Math.pow(B, 2) - Math.pow(C, 2)) / (2 * A * B)));
            System.out.println("Passed into arc cosine: " + (Math.pow(A, 2) + Math.pow(B, 2) - Math.pow(C, 2)) / (2 * A * B));
            System.out.println("A: " + A);
            System.out.println("B: " + B);
            System.out.println("C: " + C);
            System.out.println("\n");
            return 0;
        }
        return degrees;
    }

    public double getDistance(Node otherNode) {
        return Math.sqrt(Math.abs(Math.pow(otherNode.poses.get(otherNode.poses.size() - 1)[0] - this.poses.get(this.poses.size() - 1)[0], 2) + Math.pow(otherNode.poses.get(otherNode.poses.size() - 1)[1] - this.poses.get(this.poses.size() - 1)[1 ], 2)));
    }

    public String toString() {
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < poses.size(); i++) {
            builder.append("(").append(Arrays.toString(poses.get(i))).append(", ").append(Arrays.toString(poses.get(i))).append(")");
            if (i != poses.size() - 1) {
                builder.append(", ");
            }
        }
        return builder.append("\n").toString();
    }

    public boolean canBeAPart(Double[] coords) {
        if (poses.size() < 2) {
            return false;
        }
        
        double slopeOld = (poses.get(1)[1] - poses.get(0)[1]) / (poses.get(1)[0] - poses.get(0)[0]);
        double slopeNew = (coords[1] - poses.get(poses.size() - 1)[1]) / (coords[0] - poses.get(poses.size() - 1)[0]);

        if ((slopeNew > slopeOld + 1) || (slopeNew < slopeOld - 1)) {
            return false;
        }

        return this.getDistance(new Node(coords[0], coords[1])) <= 1;
    }
}
