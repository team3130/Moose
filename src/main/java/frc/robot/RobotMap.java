package frc.robot;

import edu.wpi.first.math.util.Units;

public class RobotMap {
 
public static final boolean kNavxReversed = true;

public static final String kDefaultAuto = "Default";
public static final String kCustomAuto = "My Auto";

public static final int CAN_INDEXER = 2;
public static final int CAN_MAGAZINE_MOTOR = 10;

public static final int CAN_CHASSIS_MOTOR_BACKR = 3;
public static final int CAN_CHASSIS_MOTOR_FRONTR = 4;
public static final int CAN_CHASSIS_MOTOR_BACKL = 5;
public static final int CAN_CHASSIS_MOTOR_FRONTL = 6;

public static final int CAN_PNMMODULE = 8;

public static final char CAN_SHOOTER_MOTOR = 7;
public static final int CAN_INTAKE_MOTOR = 9;

public static final double kFlywheelTicksPerRevolution = 2048;
public static final double kFlywheelRPMtoNativeUnitsScalar = RobotMap.kFlywheelTicksPerRevolution / (10.0 * 60.0);


public static final int kLimelightFilterBufferSize = 5; 
public static final double kLimelightLatencyMs = 11.0; 

public static final double kLimelightPitch =  -45;   
public static final double kLimelightYaw = 0;
public static final double kLimelightRoll = 0;   
public static final double kLimelightHeight = 0.84; 
public static final double kLimelightLength = 9.5;
public static final double kLimelightOffset = 0;
public static final double kLimelightCalibrationDist = 120.0; 

public static final double VISIONTARGETHEIGHT = 2.64; 
public static final double angleOffset = 2; 

public static final double kChassisMaxVoltage = 12.0;
public static final double trackDistance = 0.69;
public static final double kEncoderResolution = 4096;
public static final double kChassisLowGearRatio = 0.1; 
public static final double kChassisHighGearRatio = (double) 58 / 60; 
public static final double kWheelDiameter = Units.inchesToMeters(4); 
public static final double kMaxHighGearDriveSpeed = 0.8;
public static final double kMaxTurnThrottle = 0.7;
public static final double kDriveDeadband = 0.02;
public static final double kMaxRampRate = 0.7;

public static double lowGearkS = 0.615;
public static double lowGearkV = 0.0402;
public static double lowGearkA = 0.0117;

public static double highGearkS = 0.819;
public static double highGearkV = 0.0343;
public static double highGearkA = 0.00437;

  
public static final int PNM_Shift = 0;
public static final int PNM_INTAKE_ACTUATOR_LEFT = 1;

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


public static final int LST_POV_UNPRESSED = -1;
public static final int LST_POV_N = 0;
public static final int LST_POV_NE = 45;
public static final int LST_POV_E = 90;
public static final int LST_POV_SE = 135;
public static final int LST_POV_S = 180;
public static final int LST_POV_SW = 225;
public static final int LST_POV_W = 270;
public static final int LST_POV_NW = 315;

public static final int LST_AXS_LJOYSTICKX = 0;
public static final int LST_AXS_LJOYSTICKY = 1;
public static final int LST_AXS_LTRIGGER = 2;
public static final int LST_AXS_RTRIGGER = 3;
public static final int LST_AXS_RJOYSTICKX = 4;
public static final int LST_AXS_RJOYSTICKY = 5;

public static double kFlywheelP = .22;
public static double kFlywheelI = 0.0;
public static double kFlywheelD = 12;
public static double kFlywheelF = (.51*1023.0)/10650.0; 
}
