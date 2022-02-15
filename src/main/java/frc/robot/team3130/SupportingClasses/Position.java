package frc.robot.team3130.SupportingClasses;

public class Position extends Node {
    public Position(double xPos, double yPos) {
        super(xPos, yPos);
    }

    @Override
    public String nodeType() {
        return "position";
    }

}
