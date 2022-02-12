package frc.robot.team3130.SupportingClasses;

public class RedBall extends Node {

    public RedBall(double xPos, double yPos) {
        super(xPos, yPos);
    }

    @Override
    public String nodeType() {
        return "cargo_red";
    }

}
