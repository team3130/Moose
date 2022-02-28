package frc.robot.sensors;import com.kauailabs.navx.frc.AHRS;import edu.wpi.first.wpilibj.DriverStation;import edu.wpi.first.wpilibj.SPI;import edu.wpi.first.math.geometry.Rotation2d;import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;import frc.robot.RobotMap;public class Navx {private static Navx m_pInstance;public static Navx GetInstance() {if (m_pInstance == null) m_pInstance = new Navx();return m_pInstance;}private static AHRS m_navX;private static boolean m_bNavXPresent;private Navx() {try {m_navX = new AHRS(SPI.Port.kMXP);m_bNavXPresent = true;} catch (Exception ex) {String str_error = "Error instantiating navX from MXP: " + ex.getLocalizedMessage();DriverStation.reportError(str_error, true);m_bNavXPresent = false;}}public static void resetNavX(){m_navX.reset();m_navX.zeroYaw();}public static double getAngle() {if (m_bNavXPresent) return m_navX.getAngle() * (RobotMap.kNavxReversed ? -1.0 : 1.0);return -1;}public static Rotation2d getRotation() {return (m_bNavXPresent) ? m_navX.getRotation2d() : new Rotation2d(-1);}public static double getRate() {if (m_bNavXPresent) return m_navX.getRate() * (RobotMap.kNavxReversed ? -1.0 : 1.0);return -1;}public static double getHeading() {return Math.IEEEremainder(getAngle(), 360);}public static boolean getNavxPresent() {return m_bNavXPresent;}public static void outputToShuffleboard() {SmartDashboard.putNumber("NavX angle", getRotation().getDegrees());}}