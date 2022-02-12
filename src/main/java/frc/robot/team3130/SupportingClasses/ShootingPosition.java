package frc.robot.team3130.SupportingClasses;

public class ShootingPosition extends Node {

    public ShootingPosition(double xPos, double yPos) {
        super(xPos, yPos);
    }

    //TODO Run shooting program once node is reached
    public void shootBalls(int posID) {

    }

    @Override
    public String nodeType() {
        return "shooting_positiion";
    }

}
