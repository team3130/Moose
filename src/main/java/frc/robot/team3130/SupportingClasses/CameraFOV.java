package frc.robot.team3130.SupportingClasses;

public class CameraFOV {

    private Node[] FOV;

    /**
     * Sets the bounds of what the camera can see
     *
     * @param index = 0 for the far right corner
     * @param index = 1 for the near right corner
     * @param index = 2 for the near left corner
     * @param index = 3 for the far left corner
     */
    public CameraFOV(Node[] FOV) {
        this.FOV = FOV;
    }

    /**
     * @return returns true if [node] is within the FOV of the camera
     */
    public boolean isInView(Node node) {
        // (x0-x1)(y-y0) >= (y0-y1)(x-y0)
        // (x1-x2)(y-y1) >= (y1-y2)(x-y1)
        // (x2-x3)(y-y2) >= (y2-y3)(x-y2)
        // (x3-x0)(y-y3) >= (y3-y0)(x-y3)
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
