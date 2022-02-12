package frc.robot.team3130.SupportingClasses;

public class BotPosition extends Node {

    public BotPosition(double xPos, double yPos) {
        super(xPos, yPos);
    }

    @Override
    public String nodeType() {
        return "bot_position";
    }

}
