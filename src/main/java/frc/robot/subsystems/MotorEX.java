package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class MotorEX extends SubsystemBase {

// Any variables/fields used in the constructor must appear before the "INSTANCE" variable
// so that they are initialized before the constructor is called.

    //Create necessary objects
    private WPI_TalonSRX  mEX;

    //Create and define all standard data types needed
    public MotorEX() {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        mEX = new WPI_TalonSRX(RobotMap.CAN_SHOOTER_MOTOR);
    }
    public void spinMotor(double speed){
        mEX.set(speed);
    }

}

