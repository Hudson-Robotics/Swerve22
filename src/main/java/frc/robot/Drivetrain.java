package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;

public class Drivetrain {
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  private final TalonSwerve m_frontLeft = new TalonSwerve(9, 0, "FrontLeft");
  private final TalonSwerve m_frontRight = new TalonSwerve(7, 3, "FrontRight");
  private final TalonSwerve m_backLeft = new TalonSwerve(8, 2, "BackLeft");
  private final TalonSwerve m_backRight = new TalonSwerve(6, 1, "BackRight");

  private final AHRS navx = new AHRS(SPI.Port.kMXP);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation,
      m_frontRightLocation,
      m_backLeftLocation,
      m_backRightLocation);

  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, navx.getRotation2d());

  public Drivetrain() {
    Reset();
  }

  public void Reset() {
    navx.reset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, navx.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);

    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("ySpeed", ySpeed);
    SmartDashboard.putNumber("rot", rot);
    SmartDashboard.putBoolean("fieldRelative", fieldRelative);

  }

  /** Updates the field relative position of the robot. */

  public void updateOdometry() {
    m_odometry.update(navx.getRotation2d(),
        m_frontLeft.getState(), m_frontRight.getState(),
        m_backLeft.getState(), m_backRight.getState());

    SmartDashboard.putString("FrontLeftAngle", m_frontLeft.getState().toString());
    SmartDashboard.putString("FrontRightAngle", m_frontRight.getState().toString());
    SmartDashboard.putString("BackLeftAngle", m_backLeft.getState().toString());
    SmartDashboard.putString("BackRightAngle", m_backRight.getState().toString());
  }
}
