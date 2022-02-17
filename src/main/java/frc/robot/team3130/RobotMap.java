package frc.robot.team3130;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import frc.robot.team3130.SupportingClasses.Node;

public class RobotMap {
    /**
     * Navx related variables
     */
    public static final boolean kNavxReversed = true;

    /**
     * Robot related constants
     */
    public static final String kDefaultAuto = "Default";
    public static final String kCustomAuto = "My Auto";

    /**
     * Chassis
     */
    public static final double kChassisMaxVoltage = 12.0;
    // the distance between the left and the right wheels: IN METERS
    public static final double trackDistance = 0.69;
    public static final double kEncoderResolution = 4096;
    public static final double kChassisLowGearRatio = 0.1;
    public static final double kChassisHighGearRatio = 0.1;
    public static final double kWheelDiameter = Units.inchesToMeters(4); //TODO: Check if changed to six inch wheels
    public static final double kMaxHighGearDriveSpeed = 0.8;
    public static final double kMaxTurnThrottle = 0.7;
    public static final double kDriveDeadband = 0.02;
    public static final double kMaxRampRate = 0.7;

    /**
     * Auton
     */
    public static final Pose2d[] kShootingPoses = {new Pose2d(-1, -1, new Rotation2d(-1))}; //TODO: fill with actual values
    public static final double kMaxVelocityMetersPerSec = 3;
    public static final double kMaxAccelerationMetersPerSecondSq = 3;

    /**
     * PID for Chassis
     */
    public static double lowGearkS = 0.615;
    public static double lowGearkV = 0.0402;
    public static double lowGearkA = 0.0117;

    public static double highGearkS = 0.819;
    public static double highGearkV = 0.0343;
    public static double highGearkA = 0.00437;

    /**
     * CAN ID's
     */
    //TODO: FIND REAL VALUES
    public static final int CAN_PNMMODULE = 1;

    //TODO: FIND REAL VALUES
    public static final int CAN_CHASSIS_MOTOR_FRONTR = 3;
    //TODO: FIND REAL VALUES
    public static final int CAN_CHASSIS_MOTOR_FRONTL = 2;
    //TODO: FIND REAL VALUES
    public static final int CAN_CHASSIS_MOTOR_BACKR = 4;
    //TODO: FIND REAL VALUES
    public static final int CAN_CHASSIS_MOTOR_BACKL = 5;

    //TODO: FIND REAL VALUES
    public static final int PNM_Shift = 5;

    /**
     * Gamepad Button List
     */
    public static final int LST_BTN_A = 1;
    public static final int LST_BTN_B = 2;
    public static final int LST_BTN_X = 3;
    public static final int LST_BTN_Y = 4;
    public static final int LST_BTN_LBUMPER = 5;
    public static final int LST_BTN_RBUMPER = 6;
    public static final int LST_BTN_WINDOW = 7;
    public static final int LST_BTN_MENU = 8;
    public static final int LST_BTN_LJOYSTICKPRESS = 9;
    public static final int LST_BTN_RJOYSTICKPRESS = 10;

    /**
     * Gamepad POV List
     */
    public static final int LST_POV_UNPRESSED = -1;
    public static final int LST_POV_N = 0;
    public static final int LST_POV_NE = 45;
    public static final int LST_POV_E = 90;
    public static final int LST_POV_SE = 135;
    public static final int LST_POV_S = 180;
    public static final int LST_POV_SW = 225;
    public static final int LST_POV_W = 270;
    public static final int LST_POV_NW = 315;


    /**
     * Gamepad Axis List
     */
    public static final int LST_AXS_LJOYSTICKX = 0;
    public static final int LST_AXS_LJOYSTICKY = 1;
    public static final int LST_AXS_LTRIGGER = 2;
    public static final int LST_AXS_RTRIGGER = 3;
    public static final int LST_AXS_RJOYSTICKX = 4;
    public static final int LST_AXS_RJOYSTICKY = 5;
}
