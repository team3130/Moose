package frc.robot;

import edu.wpi.first.math.util.Units;

public class RobotMap {
    /**
     * Navx related variables
     */
    public static final boolean kNavxReversed = true;

    /**
     * Robot related constants
     */

    /**
     * CAN ID's
     */
    public static final int CAN_INDEXER = 2;
    public static final int CAN_MAGAZINE_MOTOR = 10;
    public static final int CAN_HOPPER_L_MOTOR = 1;
    public static final int CAN_HOPPER_R_MOTOR = 2;

    public static final int CAN_CHASSIS_MOTOR_BACKR = 3;
    public static final int CAN_CHASSIS_MOTOR_FRONTR = 4;
    public static final int CAN_CHASSIS_MOTOR_BACKL = 5;
    public static final int CAN_CHASSIS_MOTOR_FRONTL = 6;

    public static final int CAN_PNMMODULE = 8;

    public static final char CAN_SHOOTER_MOTOR = 7;
    public static final int CAN_INTAKE_MOTOR = 9;


    /**
     * Intake
     */
    public static final double kIntakeRetractTime = 0.4;


    /**
     * Shooter
     */

    //
    public static final double kFlywheelTicksPerRevolution = 2048; // Checked 2/11
    public static final double kFlywheelGearRatio = 1.0;
    public static final double kFlywheelRPMtoNativeUnitsScalar = (RobotMap.kFlywheelTicksPerRevolution / (10.0 * 60.0))/kFlywheelGearRatio;

    public static final double kIndexerGearRatio = 5.0;
    public static final double kIndexerRPMtoNativeUnitsScalar = (RobotMap.kIndexerTicksPerRevolution / (10.0 * 60.0)) / kIndexerGearRatio;
    public static final double kIndexerTicksPerRevolution = 2048; //TODO: Find real value

    public static double kFlywheelP = .22;
    public static double kFlywheelI = 0.0;
    public static double kFlywheelD = 12;
    public static double kFlywheelF = (.51*1023.0)/10650.0; // Checked 2/11, Optimal speed at 51% power

    /**
     * Limelight
     */
    public static final int kLimelightFilterBufferSize = 5; // Number of samples in input filtering window
    public static final double kLimelightLatencyMs = 11.0; // Image capture latency

    public static final double kLimelightPitch =  -45;   // Facing up is negative, in degrees Checked: 2/17
    public static final double kLimelightYaw = 0;        // Aiming bias, facing left is positive TODO: FIND FOR 2022
    public static final double kLimelightRoll = 0;       // If any, drooping to right is positive
    public static final double kLimelightHeight = 0.84;     // Height of camera aperture from the ground
    public static final double kLimelightLength = 9.5;    // Distance to the turret's rotation axis TODO: FIND FOR 2022
    public static final double kLimelightOffset = 0;    // Side offset from the turret's plane of symmetry (left+)
    public static final double kLimelightCalibrationDist = 120.0; // Exact horizontal distance between target and lens TODO: FIND FOR 2022

    public static final double VISIONTARGETHEIGHT = 2.64; // IN METERS

    public static final double angleOffset = 2; // Arbitrary value for how many degrees offset the target should be before shooting

    /**
     * Chassis
     */
    public static final double kChassisMaxVoltage = 12.0;
    // the distance between the left and the right wheels: IN METERS
    public static final double trackDistance = 0.69;
    public static final double kEncoderResolution = 2048; // checked 2/28/22
    public static final double kChassisHighGearRatio = ((double) 24/54) * ((double) 14/42); // checked 2/28/22 (For high gear)
    public static final double kChassisLowGearRatio = 0.1; //TODO: FIND VALUE
    public static double kChassisGearRatio = kChassisHighGearRatio; // default is high gear, switch when shifting
    public static final double kWheelDiameter = Units.inchesToMeters(4);
    public static final double kMaxHighGearDriveSpeed = 0.8;
    public static final double kMaxTurnThrottle = 0.9;
    public static final double kDriveDeadband = 0.02;
    public static final double kMaxRampRate = 0.7;

    // max velocity of chassis in meters per second
    public static final double kMaxVelocityMPS = 0.33;
    public static final double kMaxAccelerationMPS = 0.1;

    /**
     * PID for Chassis
     */
    public static double ChassiskP = 2.4272 / 2;
    public static double ChassiskI = 0;
    public static double ChassiskD = 0;

    public static double ChassiskS = 0.6475;
    public static double ChassiskV = 2.1534;
    public static double ChassiskA = 0.25695;

    /**
     * PNM IDs
     */
    public static final int PNM_Shift = 0;
    public static final int PNM_INTAKE_ACTUATOR_LEFT = 1;

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
