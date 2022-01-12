package frc.robot.SupportingClasses;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.HashMap;
import java.util.function.Supplier;

public class MechanismTester {
    private final HashMap<String, Supplier<Command>> map;
    private final HashMap<String, Subsystem> subsystemMap;
    private final SendableChooser<String> m_chooser;

    public MechanismTester() {
        //TODO: replace this with that are made
        String[] Names = {};

        map = new HashMap<>();
        subsystemMap = new HashMap<>();
        m_chooser = new SendableChooser<>();
    }

    public Command ToTest() {
        return map.get(m_chooser.getSelected()).get();
    }

}
