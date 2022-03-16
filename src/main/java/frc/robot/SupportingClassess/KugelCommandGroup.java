package frc.robot.SupportingClassess;

import edu.wpi.first.wpilibj2.command.*;

import java.util.ArrayList;
import java.util.Arrays;

public class KugelCommandGroup extends CommandGroupBase {
    protected ArrayList<Command> commands;
    protected int currIndex;

    public KugelCommandGroup() {
        commands = new ArrayList<>();
    }

    /**
     * Adds the given commands to the command group.
     *
     * @param commands The commands to add.
     */
    @Override
    public void addCommands(Command... commands) {
        this.commands.addAll(Arrays.asList(commands));
    }

    public void clear() {
        commands.clear();
    }

    public void removeAfter(int removeAfter) {
        while (removeAfter < commands.size()) {
            commands.remove(removeAfter);
        }
    }

    @Override
    public void initialize() {
        currIndex = 0;
        if (!commands.isEmpty()) {
            commands.get(0).initialize();
        }
    }

    @Override
    public void execute() {
        if (commands.isEmpty()) {
            return;
        }
        commands.get(currIndex).execute();

        if (commands.get(currIndex).isFinished() && currIndex != commands.size() - 1) {
            currIndex++;
            commands.get(currIndex).initialize();
        }
    }

    @Override
    public boolean isFinished() {
        return (commands.isEmpty()) || (commands.size() - 1 == currIndex && commands.get(currIndex).isFinished());
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted && !commands.isEmpty()) {
            commands.get(currIndex).end(true);
        }
        currIndex = 0;
        clear();
    }

}
