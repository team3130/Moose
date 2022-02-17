package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

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
     * Climber
     */
    public static final int kMotor_Subsystem2candevice; //add value!


    /**
     * Intake
     */
    public static final int CAN_PNMMODULE; //add value!
    public static final int PNM_INTAKE; //add value!
    public static final int CAN_Intake_MOTOR; //add value!

    /**
     * Chassis
     */
    public static Pose2d kChassisStartingPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)); //-49.37 for shoot5 auto TODO: Permanent solution for this
    public static double kChassisMaxVoltage = 12.0;
    public static double kChassisWidth = 28.0; //Chassis with from edge to edge in inches
    public static double kChassisLengthBumpers = 39.0; //FIXME
    public static double kLWheelDiameter = 6.0; // Center wheel diameter in inches
    public static double kRWheelDiameter = 6.0; // Center wheel diameter in inches
    public static double kMaxHighGearDriveSpeed = 0.8;
    public static double kMaxTurnThrottle = 0.7; // Applied on top of max drive speed
    public static double kChassisCodesPerRev = 2048;
    public static double kLChassisTicksPerInch = 1500;
    public static double kRChassisTicksPerInch = 1500;
    public static double kDriveDeadband = 0.02;
    public static double kDriveMaxRampRate = 0.7; // Minimum seconds from 0 to 100
    public static double kMPChassisP = 5.47;
    public static double kMPChassisI = 0.0;
    public static double kMPChassisD = 0.0;
    public static double kMPChassisF = 1023.0 / (92.0 * (kLChassisTicksPerInch + kRChassisTicksPerInch) / 2.0); //Checked 3/23
    public static double kChassisShiftWait = 0.07;
    public static double kChassisMPOutputDeadband = 0.01;
    public static final int CAN_RIGHTMOTORFRONT = 2;
    public static final int CAN_RIGHTMOTORREAR = 3;
    public static final int CAN_LEFTMOTORFRONT = 4;
    public static final int CAN_LEFTMOTORREAR = 5;
    public static final int PNM_SHIFT = 0;

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
