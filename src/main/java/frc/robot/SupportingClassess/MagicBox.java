package frc.robot.SupportingClassess;

import edu.wpi.first.networktables.NetworkTableEntry;

import java.util.concurrent.atomic.AtomicBoolean;

public class MagicBox {
    protected final AtomicBoolean atomicBoolean;

    protected double[] x, y;

    protected final NetworkTableEntry tableEntryX, tableEntryY;

    public MagicBox(NetworkTableEntry tableEntryX, NetworkTableEntry tableEntryY) {
        atomicBoolean = new AtomicBoolean(false);
        this.tableEntryX = tableEntryX;
        this.tableEntryY = tableEntryY;
    }

    // gets called by "Jetson Nano" (ball manager) thread
    public boolean updateBalls(double[] x, double[] y) {
        if (atomicBoolean.get()) {
            return false;
        }
        atomicBoolean.set(true);
        this.x = x;
        this.y = y;
        atomicBoolean.set(false);
        return true;
    }

    // called by rio main thread
    public boolean writeOutput() {
        if (atomicBoolean.get()) {
            return false;
        }
        atomicBoolean.set(true);
        tableEntryX.setDoubleArray(x);
        tableEntryY.setDoubleArray(y);
        atomicBoolean.set(false);
        return true;
    }

}
