<<<<<<< HEAD
package frc.robot.sensors;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

public class Navx {
    //Instance Handling
    private static Navx m_pInstance;

    private double angle = 0d;

    public static Navx GetInstance() {
        if (m_pInstance == null) m_pInstance = new Navx();
        return m_pInstance;
    }

    //Create necessary objects
    private static AHRS m_navX;


    //Create and define all standard data types needed
    private static boolean m_bNavXPresent;

    private Navx() {
        try {
            //Connect to navX Gyro on MXP port.
            m_navX = new AHRS(SPI.Port.kMXP);
            m_bNavXPresent = true;
        } catch (Exception ex) {
            //If connection fails log the error and fall back to encoder based angles.
            String str_error = "Error instantiating navX from MXP: " + ex.getLocalizedMessage();
            DriverStation.reportError(str_error, true);
            m_bNavXPresent = false;
        }
    }

    public static void resetNavX(){
        m_navX.reset();
        m_navX.zeroYaw();
    }

    /**
     * Returns the current angle of the Navx. If the Navx is not present, will return -1.
     *
     * @return angle in degrees
     */
    public static double getAngle() {
        if (m_bNavXPresent) {
            return Math.IEEEremainder((m_navX.getAngle() + 360) * (RobotMap.kNavxReversed ? -1.0 : 1.0), 720);
        }
        return -1;
    }

    /**
     * get the angle in radians
     * @return the angle as a rotation 2d (in radians)
     */
    public static Rotation2d getRotation() {
        return (m_bNavXPresent) ? m_navX.getRotation2d() : new Rotation2d(-1);
    }

    /**
     * Returns the current rate of change of the robots heading
     *
     * <p> getRate() returns the rate of change of the angle the robot is facing,
     * with a return of negative one if the gyro isn't present on the robot,
     * as calculating the rate of change of the angle using encoders is not currently being done.
     *
     * @return the rate of change of the heading of the robot in degrees per second.
     */
    public static double getRate() {
        if (m_bNavXPresent) return m_navX.getRate() * (RobotMap.kNavxReversed ? -1.0 : 1.0);
        return -1;
    }

    /**
     * Returns the current heading of the Navx. Range is wrapped onto 0 to 360
     * If the Navx is not present, will return -1.
     *
     * @return angle in degrees
     */
    public static double getHeading() {
        return Math.IEEEremainder(getAngle(), 360);
    }

    public static boolean getNavxPresent() {
        return m_bNavXPresent;
    }

    public static void outputToShuffleboard() {
        SmartDashboard.putNumber("NavX angle", getRotation().getDegrees());
    }
}
=======
package frc.robot.sensors;import com.kauailabs.navx.frc.AHRS;import edu.wpi.first.wpilibj.DriverStation;import edu.wpi.first.wpilibj.SPI;import edu.wpi.first.math.geometry.Rotation2d;import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;import frc.robot.RobotMap;public class Navx {private static Navx m_pInstance;public static Navx GetInstance() {if (m_pInstance == null) m_pInstance = new Navx();return m_pInstance;}private static AHRS m_navX;private static boolean m_bNavXPresent;private Navx() {try {m_navX = new AHRS(SPI.Port.kMXP);m_bNavXPresent = true;} catch (Exception ex) {String str_error = "Error instantiating navX from MXP: " + ex.getLocalizedMessage();DriverStation.reportError(str_error, true);m_bNavXPresent = false;}}public static void resetNavX(){m_navX.reset();m_navX.zeroYaw();}public static double getAngle() {if (m_bNavXPresent) return m_navX.getAngle() * (RobotMap.kNavxReversed ? -1.0 : 1.0);return -1;}public static Rotation2d getRotation() {return (m_bNavXPresent) ? m_navX.getRotation2d() : new Rotation2d(-1);}public static double getRate() {if (m_bNavXPresent) return m_navX.getRate() * (RobotMap.kNavxReversed ? -1.0 : 1.0);return -1;}public static double getHeading() {return Math.IEEEremainder(getAngle(), 360);}public static boolean getNavxPresent() {return m_bNavXPresent;}public static void outputToShuffleboard() {SmartDashboard.putNumber("NavX angle", getRotation().getDegrees());}}
>>>>>>> 14bc2c440138565f9b13fd5f6896dd93bc0ec253
