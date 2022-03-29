package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.systems.Climber;
import frc.robot.systems.Index;
import frc.robot.systems.Intake;
import frc.robot.systems.LimeLight;
import frc.robot.systems.Shooter;
import frc.robot.systems.DriveTrain.Drivetrain;

public class Robot extends TimedRobot {
  private Alliance alliance;

  private final XboxController m_controller = new XboxController(0);

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(1);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(1);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(1);

  private boolean shooterRun;

  private final LimeLight limeLight = LimeLight.getInstance();
  private final Shooter shooter = Shooter.getInstance();
  private final Climber climber = Climber.getInstance();
  private final Index index = Index.getInstance();
  private final Intake intake = Intake.getInstance();
  private final Drivetrain m_swerve = Drivetrain.getInstance();

  private final PneumaticHub PnueHub = new PneumaticHub(22);

  private double ptHigh;
  private double ptWork;
  private double compCurrent;

  private final DigitalInput lsShooterHome = new DigitalInput(0);
  private final DigitalInput lsClimbLeft = new DigitalInput(1);
  private final DigitalInput lsClimbRight = new DigitalInput(2);

  private Timer timer = new Timer();

  @Override
  public void robotPeriodic() {
    alliance = DriverStation.getAlliance();

    ptHigh = PnueHub.getPressure(0);
    ptWork = PnueHub.getPressure(1);
    compCurrent = PnueHub.getCompressorCurrent();

    SmartDashboard.putNumber("PT High", ptHigh);
    SmartDashboard.putNumber("PT Work", ptWork);
    SmartDashboard.putNumber("Air Comp Amps", compCurrent);

    limeLight.updateMeasurements();

    m_swerve.updateOdometry();

  }

  @Override
  public void autonomousInit() {
    timer.reset();
    timer.start();
  }

  @Override
  public void autonomousPeriodic() {
    // driveWithJoystick(false);
    /*
     * if (timer.get() < 2) {
     * shooter.set(TalonFXControlMode.PercentOutput, -.7);
     * } else if (timer.get() < 5) {
     * indexTop.set(-.3);
     * indexBottom.set(.3);
     * } else if (timer.get() < 8) {
     * var xSpeed = .3 * Drivetrain.kMaxSpeed;
     * m_swerve.drive(xSpeed, 0, 0, false);
     * } else {
     * shooter.set(TalonFXControlMode.PercentOutput, 0);
     * indexTop.set(0);
     * indexBottom.set(0);
     * m_swerve.drive(0, 0, 0, false);
     * }
     */
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {

    if (m_controller.getLeftBumperPressed() && m_controller.getRightBumperPressed()) {
      m_swerve.Reset();
    }

    boolean yButtonPress = m_controller.getYButtonPressed();

    if (yButtonPress) {
      shooterRun = !shooterRun;
    }
    drive(true);
    intake.intake(m_controller, shooterRun);
    index.index(m_controller, shooterRun);
    shooter.shoot(m_controller, shooterRun, lsShooterHome.get(), alliance);
    climber.climb(m_controller, lsClimbLeft.get(), lsClimbRight.get());
  }

  private void drive(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.05))
        * Drivetrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed = -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.05))
        * Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.05))
        * Drivetrain.kMaxAngularSpeed;

    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
  }

}
