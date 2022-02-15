package frc.robot.team3130.SupportingClasses;

public class CameraFOV {

    private Position[] FOV;

    /**
     * Sets the bounds of what the camera can see
     *
     * @param FOV[0] = far right corner
     * @param FOV[1] = near right corner
     * @param FOV[2] = near left corner
     * @param FOV[3] = far left corner
     */
    public CameraFOV(Position[] FOV) {
        this.FOV = FOV;
    }

    /**
     * @return returns true if [node] is within the FOV of the camera
     */
    public boolean isInView(Node node) {
        // Uses the slopes between the corners of the camera FOV
        if ((node.getY() - FOV[0].getY()) >= (FOV[0].getY() - FOV[1].getY())
                / (FOV[0].getX() - FOV[1].getX()) * (node.getX() - FOV[0].getX())
            && (node.getY() - FOV[1].getY()) >= (FOV[1].getY() - FOV[2].getY())
                / (FOV[1].getX() - FOV[2].getX()) * (node.getX() - FOV[1].getX())
            && (node.getY() - FOV[2].getY()) >= (FOV[2].getY() - FOV[3].getY())
                / (FOV[2].getX() - FOV[3].getX()) * (node.getX() - FOV[2].getX())
            && (node.getY() - FOV[3].getY()) >= (FOV[3].getY() - FOV[0].getY())
                / (FOV[3].getX() - FOV[0].getX()) * (node.getX() - FOV[3].getX())) {
            return true;
        }
        return false;
    }
}
