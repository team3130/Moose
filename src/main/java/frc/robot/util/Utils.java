package frc.robot.util;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Utils {

    /**
     * Prevent this class from being instantiated.
     */
    private Utils() {
    }

    /**
     * Limits the given input to the given magnitude.
     */
    public static double limit(double v, double maxMagnitude) { return limit(v, -maxMagnitude, maxMagnitude);
    }
    /**
     * Limits the given input between the given minimum and maximum.
     */
    public static double limit(double v, double min, double max) {
        return Math.min(max, Math.max(min, v));
    }

    /**
     * Returns 0.0 if the given value is within the specified range around zero. The remaining range
     * between the deadband and 1.0 is scaled from 0.0 to 1.0.
     *
     * @param val    value to trim
     * @param deadband range around zero
     */
    public static double applyDeadband(double val, double deadband){
        if(Math.abs(val) > deadband){
            if(val > 0.0){
                return (val - deadband)/ (1.0 - deadband);
            } else {
                return (val + deadband)/ (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    /**
     * Configure motion magic parameters of a Talon motor controller
     * @param acceleration maximum/target acceleration
     * @param cruiseVelocity cruise velocity
     */
    public static void configMotionMagic(WPI_TalonSRX _talon, int acceleration, int cruiseVelocity){
        _talon.configMotionCruiseVelocity(cruiseVelocity, 0);
        _talon.configMotionAcceleration(acceleration, 0);
    }

    /**
     * Configure the PIDF values of a Talon motor controller
     * @param _talon the Talon motor controller
     * @param kP
     * @param kI
     * @param kD
     * @param kF
     */
    public static void configPIDF(WPI_TalonSRX _talon, double kP, double kI, double kD, double kF) {
        _talon.config_kP(0, kP, 0);
        _talon.config_kI(0, kI, 0);
        _talon.config_kD(0, kD, 0);
        _talon.config_kF(0, kF, 0);
    }

    /**
     * Configure the PIDF values of a Talon motor controller
     * @param _talon the Talon motor controller
     * @param kP
     * @param kI
     * @param kD
     * @param kF
     */
    public static void configPIDF(WPI_TalonFX _talon, double kP, double kI, double kD, double kF) {
        _talon.config_kP(0, kP, 0);
        _talon.config_kI(0, kI, 0);
        _talon.config_kD(0, kD, 0);
        _talon.config_kF(0, kF, 0);
    }

}
