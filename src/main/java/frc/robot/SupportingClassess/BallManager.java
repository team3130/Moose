package frc.robot.SupportingClassess;

public class BallManager {
    // 22 is the buffer size (there should only be 11 on the field but meh)
    /**
     * General structure of the array, the quickest ball will have its index cached,
     * the quickest two ball will have its index cached in an array of size two
     * then the overall array will be ordered for thew the quickest path to every known ball
     * This system will have to be dynamic,
     * so that we can find the quickest path for the next n balls into the future
     *
     * This system is designed in this way so that a single async method can handle all the logic
     */
    protected final Ball[] balls = new Ball[22];
    protected int highest = 0;

    protected int quickestOneBall = 0;
    protected int[] quickestTwoBall = new int[] {0, 0};



    // buffer of size 20 for balls to add
    protected final Ball[] toAdd = new Ball[22];
    protected int highestToAddIndex = 0;

    /*
            double[] range1;
        // find good shooting spots from shooter curves, same approach as a heap if you think about it (just without the ds)
        double lowest = 100;
        // the lowest is a lower bound not a viable place to shoot from
        double secondLowest = 100;
        double highest = 0;
        // the highest is an upper bound not a viable place to shoot from
        double secondHighest = 0;
        // thank god this is on innit otherwise it would destroy RIO (side note: why are if statements so frickin slow)
        try (BufferedReader br = new BufferedReader(new FileReader(shooterCurvePath))) {
            for (String line; (line = br.readLine()) != null; ) {
                if (!line.equals("")) {
                    // gets the distance from the current line
                    double curr = Double.parseDouble(line.split(",")[0]);
                    if (curr < lowest) {
                        // the lowest becomes the second lowest
                        secondLowest = lowest;
                        lowest = curr;
                    }
                    // this should be in between the lowest and second lowest
                    else if (curr < secondLowest) {
                        secondLowest = curr;
                    }
                    // rinse and repeat but swapped inequality and for highest
                    if (curr > highest) {
                        secondHighest = highest;
                        highest = curr;
                    }
                    else if (curr > secondHighest) {
                        secondHighest = curr;
                    }
                }
            }
            if (secondLowest != secondHighest && lowest != secondLowest && secondHighest != highest && lowest != highest) {
                range1 = new double[] {secondLowest, secondHighest};
            }
            else {
                range1 = new double[] {2.4, 4.56};
            }
        }
        catch (IOException e) {
            range1 = new double[]{2.4, 4.56};
            DriverStation.reportError("Could not read csv in BallManager, defaulting", RobotMap.debug);
        }
        range = range1;
     */

    public Ball getQuickestOne() {
        return balls[quickestOneBall];
    }

    public Ball[] getQuickestTwo() {
        return new Ball[] {balls[quickestTwoBall[0]], balls[quickestTwoBall[1]]};
    }

    public Ball[] getPath() {
        // protecting the array
        return balls.clone();
    }

    public void clear() {
        // when you're making a destructor in Java
        for (Ball ball : balls) {
            ball = null;
        }
    }

    // this is synced with rio, so it merely queues the addition of balls
    public void addBall(Ball... balls) {
        // the slowest thing that gets ran on rio, but is also pretty unavoidable
        for(Ball ball : balls) {toAdd[highestToAddIndex++] = ball;}
    }

}
