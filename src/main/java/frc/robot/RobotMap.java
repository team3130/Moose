package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.robot.utils.Utils;

public class RobotMap {
    public static final int DIO_FEEDERBEAM = 1;
    // If you start running out of RAM change everything in here to a final char

    public static boolean debug = true; // whether we should print stack traces
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
    // indexer
    public static final int CAN_INDEXER = 2;

    // magazine/hopper
    public static final int CAN_MAGAZINE_CENTER_MOTOR = 9;
    public static final int CAN_MAGAZINE_LEFT_MOTOR = 15;
    public static final int CAN_MAGAZINE_RIGHT_MOTOR = 12;

    // drivetrain
    public static final int CAN_CHASSIS_MOTOR_BACKR = 3;
    public static final int CAN_CHASSIS_MOTOR_FRONTR = 4;
    public static final int CAN_CHASSIS_MOTOR_BACKL = 5;
    public static final int CAN_CHASSIS_MOTOR_FRONTL = 6;

    // PNM
    public static final int CAN_PNMMODULE = 8;

    // Shooter
    public static final char CAN_SHOOTER_MOTOR = 7;
    public static final int CAN_SHOOTER_UPPER_MOTOR = 13;
    public static final int CAN_HOOD_MOTOR = 14;

    // Intake
    public static final int CAN_INTAKE_MOTOR = 10;

    // Climber
    public static final int CAN_CLIMBER_LEFT = 16;
    public static final int CAN_CLIMBER_RIGHT = 11;

    /**
     * Intake
     */
    public static final double kIntakeRetractTime = 0.4;

    /**
     * Shooter
     */

    //
    public static final double kFalconTicksPerRevolution = 2048; // Checked 2/11
    public static final double kFlywheelGearRatio = 1.0;
    public static final double kTopShooterGearRatio = 2;
    public static final double kFlywheelRPMtoNativeUnitsScalar = (RobotMap.kFalconTicksPerRevolution / (10.0 * 60.0))/kFlywheelGearRatio;
    public static final double kTopShooterRPMToNativeUnitsScalar = (RobotMap.kFalconTicksPerRevolution / (10 * 60)) / kTopShooterGearRatio;

    public static final double kIndexerGearRatio = 5.0;
    public static final double kIndexerRPMtoNativeUnitsScalar = (RobotMap.kIndexerTicksPerRevolution / (10.0 * 60.0)) / kIndexerGearRatio;
    public static final double kIndexerTicksPerRevolution = 2048; //TODO: Find real value

    public static double kFlywheelP = .22;
    public static double kFlywheelI = 0.0;
    public static double kFlywheelD = 12;
    public static double kFlywheelF = (.53*1023.0)/10650.0; // Checked 2/11/20, Optimal speed at 51% power

    public static double kFlywheelHoodP = .22;
    public static double kFlywheelHoodI = 0.0;
    public static double kFlywheelHoodD = 12;
    public static double kFlywheelHoodF = (.75*1023.0)/10650.0; // Checked 2/11/20, Optimal speed at 51% power

    public static double kHoodP = 0.125;
    public static double kHoodI = 0.0;
    public static double kHoodD = 0;
    public static double kHoodF = 0;

    /**
     * Limelight
     */
    public static final int kLimelightFilterBufferSize = 5; // Number of samples in input filtering window
    public static final double kLimelightLatencyMs = 11.0; // Image capture latency

    public static final double kLimelightPitch =  -38.5;   // Facing up is negative, in degrees Checked: 2/17
    public static final double kLimelightYaw = 0;        // Aiming bias, facing left is positive TODO: FIND FOR 2022
    public static final double kLimelightRoll = 0;       // If any, drooping to right is positive
    public static final double kLimelightHeight = 0.8255;     // Height of camera aperture from the ground
    public static final double kLimelightLength = 0;    // Distance to the turret's rotation axis TODO: FIND FOR 2022
    public static final double kLimelightOffset = 0;    // Side offset from the turret's plane of symmetry (left+)
    public static final double kLimelightCalibrationDist = Units.inchesToMeters(120.0); // Exact horizontal distance between target and lens TODO: FIND FOR 2022
    public static final double ShootingSweetSpot = 4; //TODO: Find sweet spot

    public static final double VISIONTARGETHEIGHT = 2.64; // IN METERS

    public static final double angleOffset = 2; // Arbitrary value for how many degrees offset the target should be before shooting

    /**
     * Chassis
     */
    public static final double kChassisMaxVoltage = 12.0;
    // the distance between the left and the right wheels: IN METERS
    public static final double trackDistance = Units.inchesToMeters(27.089); //taken 3/20/22 COMP
    public static final double kEncoderResolution = 2048; // checked 2/28/22
    public static final double kChassisHighGearRatio = ((double) 24/54) * ((double) 14/42); // checked 2/28/22 (For high gear)
    public static final double kChassisLowGearRatio = 0.1; //TODO: FIND VALUE
    public static double kChassisGearRatio = kChassisHighGearRatio; // default is high gear, switch when shifting
    public static final double kWheelDiameter = 0.100305;//taken 3/20/22 COMP
    public static final double kMaxHighGearDriveSpeed = 0.8;
    public static final double kMaxTurnThrottle = 0.9;
    public static final double kDriveDeadband = 0.02;
    public static final double kMaxRampRate = 0.7;
    public static final double kChassisEncoderError = 1; //TODO: Still needs to be determined for comp bot

    public static double ChassisSpinKP = 0.0125;
    public static double ChassisSpinKI = 0;
    public static double ChassisSpinKD = 0;

    // max velocity of chassis in meters per second
    public static final double kMaxVelocityMPS = 2.25;
    public static final double kMaxAccelerationMPS = 1.5;

    /**
     * PID for Chassis
     */
            // solution found, cry harder
    public static double LChassiskP = 3.186; //Practice bot 3/10/22
    public static double LChassiskI = 1.5;
    public static double LChassiskD = 0;

    public static double RChassiskP = 2.6838;
    public static double RChassiskI = 1.33;
    public static double RChassiskD = 0;

    public static double ChassiskS = 0.66218;
    public static double ChassiskV = 2.3117;
    public static double ChassiskA = 0.33604;

    /**
     * PNM IDs
     */
    public static final int PNM_Shift = 0;
    public static final int PNM_INTAKE_ACTUATOR_LEFT = 1;
    public static final int PNM_CLIMBER_ACTUATOR = 2;

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
