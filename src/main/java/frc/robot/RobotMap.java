<<<<<<< HEAD
package frc.robot;

import edu.wpi.first.math.util.Units;

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
    public static final int CAN_SHOOTER_LEFT = 7;
    public static final int CAN_SHOOTER_RIGHT = 13;
    public static final int CAN_HOOD_MOTOR = 14;

    // Intake
    public static final int CAN_INTAKE_MOTOR = 10;

    // Climber
    public static final int CAN_CLIMBER_LEFT = 17;
    public static final int CAN_CLIMBER_RIGHT = 29;
    public static final int RIGHT_LIMITSWITCH = 2;
    public static final int LEFT_LIMITSWITCH = 3;

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
    public static final double kTopShooterGearRatio = 1;
    public static final double kFlywheelRPMtoNativeUnitsScalar = (RobotMap.kFalconTicksPerRevolution / (10.0 * 60.0))/kFlywheelGearRatio;
    public static final double kTopShooterRPMToNativeUnitsScalar = (RobotMap.kFalconTicksPerRevolution / (10 * 60)) / kTopShooterGearRatio;

    public static final double kIndexerGearRatio = 5.0;
    public static final double kIndexerRPMtoNativeUnitsScalar = (RobotMap.kIndexerTicksPerRevolution / (10.0 * 60.0)) / kIndexerGearRatio;
    public static final double kIndexerTicksPerRevolution = 2048;

    // this might not be actual rotations, but it doesn't matter
    public static final double HoodScalarToRotations = 4096*100; // <- mostly arbitrary numbers to lower the size of the position number

    public static double kFlywheelP = 0.02; // 0.02;
    public static double kFlywheelI = 0.000125;
    public static double kFlywheelD = 0.125; //0

    public static double WPItoCTREFeedForwardConversion = ((1023.0/12.0) * 10.0) / 2048.0; // assuming gains are in Rotations/Sec

    public static double flyWheelkS = 0 ;//0.5816;
    public static double flyWheelkV = 0.0655; //0.0655;
    public static double flyWheelkA = 0; //0.0082804;

    public static double kHoodP = 1.53
            ;
    public static double kHoodI = 0.0;
    public static double kHoodD = 0;
    public static double kHoodV = 0;


    public static double kClimberP = 0.00005;
    public static double kClimberI = 0;
    public static double kClimberD = 0;
    public static double kClimberS = 1;

    /**
     * Limelight
     */
    public static final int kLimelightFilterBufferSize = 5; // Number of samples in input filtering window
    public static final double kLimelightLatencyMs = 11.0; // Image capture latency

    public static final double kLimelightPitch =  -37.9;   // Facing up is negative, in degrees Checked: 2/17
    public static final double kLimelightYaw = -1.75;        // Aiming bias, facing left is positive
    public static final double kLimelightRoll = 0;       // If any, drooping to right is positive
    public static final double kLimelightHeight = 0.8255;     // Height of camera aperture from the ground
    public static final double kLimelightLength = 0;    // Distance to the turret's rotation axis
    public static final double kLimelightOffset = 0;    // Side offset from the turret's plane of symmetry (left+)
    public static final double kLimelightCalibrationDist = Units.inchesToMeters(120.0); // Exact horizontal distance between target and lens

    public static final double VISIONTARGETHEIGHT = 2.64; // IN METERS

    /**
     * Chassis
     */
    public static final double kChassisMaxVoltage = 10.0;
    // the distance between the left and the right wheels: IN METERS
    public static final double trackDistance = Units.inchesToMeters(27.089); //taken 3/20/22 COMP
    public static final double kEncoderResolution = 2048; // checked 2/28/22
    public static final double kChassisHighGearRatio = ((double) 24/54) * ((double) 14/42); // checked 2/28/22 (For high gear)
    public static double kChassisGearRatio = kChassisHighGearRatio; // default is high gear, switch when shifting
    public static final double kWheelDiameter = Units.inchesToMeters(3.955);//taken 3/20/22 COMP
    public static double kMaxTurnThrottle = 0.6;
    public static double kMaxHighGearDriveSpeed = 0.8;
    public static double kMaxOutreachDriveSpeed = 0.55;
    public static final double kDriveDeadband = 0.02;
    public static final double kChassisEncoderError = 1.0934926;
    public static double kMaxRampRate = 0.7;

    public static double ChassisSpinKP = 0.012;
    public static double ChassisSpinKI = 0.01;
    public static double ChassisSpinKD = 0.002;


    // max velocity of chassis in meters per second
    public static final double kMaxVelocityMPS = 1.5;
    public static final double kMaxAccelerationMPS = 1;

    /**
     * PID for Chassis
     */
            // solution found, cry harder
    public static double LChassiskP = 2.7278; //MURA
    public static double LChassiskI = 1.5;
    public static double LChassiskD = 0;

    public static double RChassiskP = 2.6834;
    public static double RChassiskI = 1.33;
    public static double RChassiskD = 0;

    public static double ChassiskS = 0.67321;
    public static double ChassiskV = 1.924;
    public static double ChassiskA = 0.60662;


    /**
     * PNM IDs
     */
    public static final int PNM_Shift = 0;
    public static final int PNM_INTAKE_ACTUATOR_LEFT = 1;
    public static final int PNM_CLIMBER_ACTUATOR = 2;
    public static final int PNM_CHASSIS_COOLER = 3;
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
=======
package frc.robot;import edu.wpi.first.math.util.Units;public class RobotMap {public static final boolean kNavxReversed = true;public static final String kDefaultAuto = "Default";public static final String kCustomAuto = "My Auto";public static final int CAN_INDEXER = 2;public static final int CAN_MAGAZINE_MOTOR = 10;public static final int CAN_CHASSIS_MOTOR_BACKR = 3;public static final int CAN_CHASSIS_MOTOR_FRONTR = 4;public static final int CAN_CHASSIS_MOTOR_BACKL = 5;public static final int CAN_CHASSIS_MOTOR_FRONTL = 6;public static final int CAN_PNMMODULE = 8;public static final char CAN_SHOOTER_MOTOR = 7;public static final int CAN_INTAKE_MOTOR = 9;public static final double kFlywheelTicksPerRevolution = 2048;public static final double kFlywheelRPMtoNativeUnitsScalar = RobotMap.kFlywheelTicksPerRevolution / (10.0 * 60.0);public static final int kLimelightFilterBufferSize = 5;public static final double kLimelightLatencyMs = 11.0;public static final double kLimelightPitch =  -45;public static final double kLimelightYaw = 0;public static final double kLimelightRoll = 0;public static final double kLimelightHeight = 0.84;public static final double kLimelightLength = 9.5;public static final double kLimelightOffset = 0;public static final double kLimelightCalibrationDist = 120.0;public static final double VISIONTARGETHEIGHT = 2.64; public static final double angleOffset = 2; public static final double kChassisMaxVoltage = 12.0;public static final double trackDistance = 0.69;public static final double kEncoderResolution = 4096;public static final double kChassisLowGearRatio = 0.1; public static final double kChassisHighGearRatio = (double) 58 / 60; public static final double kWheelDiameter = Units.inchesToMeters(4); public static final double kMaxHighGearDriveSpeed = 0.8;public static final double kMaxTurnThrottle = 0.7;public static final double kDriveDeadband = 0.02;public static final double kMaxRampRate = 0.7;public static double lowGearkS = 0.615;public static double lowGearkV = 0.0402;public static double lowGearkA = 0.0117;public static double highGearkS = 0.819;public static double highGearkV = 0.0343;public static double highGearkA = 0.00437;public static final int PNM_Shift = 0;public static final int PNM_INTAKE_ACTUATOR_LEFT = 1;public static final int LST_BTN_A = 1;public static final int LST_BTN_B = 2;public static final int LST_BTN_X = 3;public static final int LST_BTN_Y = 4;public static final int LST_BTN_LBUMPER = 5;public static final int LST_BTN_RBUMPER = 6;public static final int LST_BTN_WINDOW = 7;public static final int LST_BTN_MENU = 8;public static final int LST_BTN_LJOYSTICKPRESS = 9;public static final int LST_BTN_RJOYSTICKPRESS = 10;public static final int LST_POV_UNPRESSED = -1;public static final int LST_POV_N = 0;public static final int LST_POV_NE = 45;public static final int LST_POV_E = 90;public static final int LST_POV_SE = 135;public static final int LST_POV_S = 180;public static final int LST_POV_SW = 225;public static final int LST_POV_W = 270;public static final int LST_POV_NW = 315;public static final int LST_AXS_LJOYSTICKX = 0;public static final int LST_AXS_LJOYSTICKY = 1;public static final int LST_AXS_LTRIGGER = 2;public static final int LST_AXS_RTRIGGER = 3;public static final int LST_AXS_RJOYSTICKX = 4;public static final int LST_AXS_RJOYSTICKY = 5;public static double kFlywheelP = .22;public static double kFlywheelI = 0.0;public static double kFlywheelD = 12;public static double kFlywheelF = (.51*1023.0)/10650.0;}
>>>>>>> 14bc2c440138565f9b13fd5f6896dd93bc0ec253
