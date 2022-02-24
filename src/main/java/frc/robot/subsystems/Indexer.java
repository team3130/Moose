package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Indexer extends SubsystemBase {

// Any variables/fields used in the constructor must appear before the "INSTANCE" variable
// so that they are initialized before the constructor is called.

    //Create necessary objects
    private WPI_TalonSRX m_indexer;
    private ShuffleboardTab tab = Shuffleboard.getTab("Motor");
    private NetworkTableEntry indexerReadPercent = tab.add("IndexerRead%", 0).getEntry();
    private NetworkTableEntry indexerVoltageOut = tab.add("Indexer Voltage", 0).getEntry();

    //Create and define all standard data types needed
    public Indexer() {
        m_indexer = new WPI_TalonSRX(RobotMap.CAN_INDEXER);
    }

    public void setSpeed(double sped) {
        m_indexer.set(sped);
    }

    public void setPercent(double percent){
        m_indexer.set(ControlMode.PercentOutput, percent);
    }

    public double getPercentFromShuffleboard(){
        return indexerReadPercent.getDouble(0);
    }

    public void writeOutput() {
        indexerVoltageOut.setNumber(m_indexer.getMotorOutputVoltage());
    }

    @Override
    public void periodic() {
        writeOutput();
    }



}

