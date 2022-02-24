package frc.robot;

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
    public static final int KMAGAZINEMOTORCANID = 4; //TODO give this a real value

    public static final double kMotorGearRatio = 1;
    public static final double kMotorTicksPerRevolution = 2048;


    /**
     * CAN ID's
     */

    public static final char CAN_SHOOTER_MOTOR = 5;
    public static final int CAN_INDEXER = 3;


    public static final double kFlywheelTicksPerRevolution = 2048; // Checked 2/11
    public static final double kFlywheelRPMtoNativeUnitsScalar = RobotMap.kFlywheelTicksPerRevolution / (10.0 * 60.0);

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

    public static double kFlywheelP = .22;
    public static double kFlywheelI = 0.0;
    public static double kFlywheelD = 12;
    public static double kFlywheelF = (.51*1023.0)/10650.0; // Checked 2/11, Optimal speed at 51% power
}
