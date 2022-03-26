package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Robot extends TimedRobot {
  private Alliance alliance;

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color kBlueBall = new Color(0.17, 0.41, 0.41);
  private final Color kRedBall = new Color(0.51, 0.35, 0.15);

  private final XboxController m_controller = new XboxController(0);
  private final Drivetrain m_swerve = new Drivetrain();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(1);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(1);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(1);

  private final CANSparkMax climbLeft = new CANSparkMax(14, MotorType.kBrushless);
  private final CANSparkMax climbRight = new CANSparkMax(9, MotorType.kBrushless);

  private final CANSparkMax indexTop = new CANSparkMax(10, MotorType.kBrushless);
  private final CANSparkMax indexBottom = new CANSparkMax(11, MotorType.kBrushless);

  private final CANSparkMax shooterAngle = new CANSparkMax(5, MotorType.kBrushless);
  private final TalonFX shooter = new TalonFX(3);

  private final TalonSRX intake = new TalonSRX(13);

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  private final PneumaticHub PnueHub = new PneumaticHub(22);
  private final DoubleSolenoid climbCylinders = PnueHub.makeDoubleSolenoid(0, 1);
  private final DoubleSolenoid intLeftCylinders = PnueHub.makeDoubleSolenoid(2, 3);
  private final DoubleSolenoid intRightCylinders = PnueHub.makeDoubleSolenoid(6, 7);
  private double ptHigh;
  private double ptWork;
  private double compCurrent;

  private final DigitalInput lsShooterHome = new DigitalInput(0);
  private final DigitalInput lsClimbLeft = new DigitalInput(1);
  private final DigitalInput lsClimbRight = new DigitalInput(2);

  private int proximity;
  private Color detectedColor;

  private Timer timer = new Timer();

  @Override
  public void robotInit() {
    m_colorMatcher.addColorMatch(kBlueBall);
    m_colorMatcher.addColorMatch(kRedBall);
  }

  @Override
  public void robotPeriodic() {
    alliance = DriverStation.getAlliance();
    proximity = m_colorSensor.getProximity();
    double IR = m_colorSensor.getIR();
    detectedColor = m_colorSensor.getColor();

    ptHigh = PnueHub.getPressure(0);
    ptWork = PnueHub.getPressure(1);
    compCurrent = PnueHub.getCompressorCurrent();

    SmartDashboard.putNumber("Proximity", proximity);

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", IR);
    SmartDashboard.putNumber("PT High", ptHigh);
    SmartDashboard.putNumber("PT Work", ptWork);
    SmartDashboard.putNumber("Air Comp Amps", compCurrent);

    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

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
    if (timer.get() < 2) {
      shooter.set(TalonFXControlMode.PercentOutput, -.7);
    } else if (timer.get() < 5) {
      indexTop.set(-.3);
      indexBottom.set(.3);
    } else if (timer.get() < 8) {
      var xSpeed = .3 * Drivetrain.kMaxSpeed;
      m_swerve.drive(xSpeed, 0, 0, false);
    } else {
      shooter.set(TalonFXControlMode.PercentOutput, 0);
      indexTop.set(0);
      indexBottom.set(0);
      m_swerve.drive(0, 0, 0, false);
    }
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    if (m_controller.getLeftBumperPressed() && m_controller.getRightBumperPressed()) {
      m_swerve.Reset();
    }

    // drive(true);
    intake();
    index();
    shoot();
    climb();
  }

  private void drive(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(-m_controller.getLeftY(), 0.05))
        * Drivetrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed = -m_yspeedLimiter.calculate(MathUtil.applyDeadband(-m_controller.getLeftX(), 0.05))
        * Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(-m_controller.getRightX(), 0.05))
        * Drivetrain.kMaxAngularSpeed;

    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
  }

  private void intake() {
    double leftTrigger = m_controller.getLeftTriggerAxis();
    double rightTrigger = m_controller.getRightTriggerAxis();
    boolean leftTriggerPressed;
    boolean rightTriggerPressed;

    if (leftTrigger > .5) {
      leftTriggerPressed = true;
    } else {
      leftTriggerPressed = false;
    }

    if (rightTrigger > .5) {
      rightTriggerPressed = true;
    } else {
      rightTriggerPressed = false;
    }

    SmartDashboard.putBoolean("Left Trigger Bool", leftTriggerPressed);
    SmartDashboard.putNumber("Left Trigger Position", leftTrigger);
    SmartDashboard.putBoolean("Right Trigger Bool", rightTriggerPressed);
    SmartDashboard.putNumber("Right Trigger Position", rightTrigger);

    if (shooterRun) {
      intake.set(TalonSRXControlMode.PercentOutput, -.7);
      intLeftCylinders.set(Value.kReverse);
      intRightCylinders.set(Value.kReverse);
    } else if (rightTriggerPressed) {
      intLeftCylinders.set(Value.kForward);
      intRightCylinders.set(Value.kForward);
    } else {
      intake.set(TalonSRXControlMode.PercentOutput, 0);
      intLeftCylinders.set(Value.kOff);
      intRightCylinders.set(Value.kOff);
    }
  }

  private void index() {
    boolean aButton = m_controller.getAButton();
    boolean bButton = m_controller.getBButton();
    boolean xButton = m_controller.getXButton();
    boolean prox;

    if (proximity >= 200) {
      prox = true;
    } else {
      prox = false;
    }

    SmartDashboard.putBoolean("A Button", aButton);
    SmartDashboard.putBoolean("B Button", bButton);
    SmartDashboard.putBoolean("X Button", xButton);
    SmartDashboard.putBoolean("Prox", prox);

    if (!aButton & xButton) {
      indexTop.set(.3);
      indexBottom.set(-.3);
    } else if (aButton & !prox) {
      indexTop.set(-.3);
      indexBottom.set(.3);
    } else if (bButton & shooterRun) {
      indexTop.set(-.3);
      indexBottom.set(.3);
    } else {
      indexTop.set(0);
      indexBottom.set(0);
    }

  }

  private boolean shooterRun;

  private void shoot() {
    boolean rightBumper = m_controller.getRightBumper();
    boolean lefttBumper = m_controller.getLeftBumper();
    boolean yButtonPress = m_controller.getYButtonPressed();

    SmartDashboard.putBoolean("Right Bumper", rightBumper);
    SmartDashboard.putBoolean("Left Bumper", lefttBumper);
    SmartDashboard.putBoolean("Y Button", yButtonPress);

    if (rightBumper & lsShooterHome.get()) {
      shooterAngle.set(-.1);
    } else if (rightBumper & lsShooterHome.get()) {
      shooterAngle.set(0);
      m_controller.setRumble(RumbleType.kLeftRumble, 1);
      m_controller.setRumble(RumbleType.kRightRumble, 1);
    } else if (lefttBumper) {
      shooterAngle.set(.1);
    } else {
      shooterAngle.set(0);
      m_controller.setRumble(RumbleType.kLeftRumble, 0);
      m_controller.setRumble(RumbleType.kRightRumble, 0);
    }

    String colorString = "Unknown";
    Boolean colorAccept = false;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    // if (proximity >= 200) {
    if (match.color == kBlueBall) {
      colorString = "Blue";
      if (alliance == Alliance.Blue) {
        colorAccept = true;
      }
    } else if (match.color == kRedBall) {
      colorString = "Red";
      if (alliance == Alliance.Red) {
        colorAccept = true;
      }
    }
    // }

    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Color Detected", colorString);
    SmartDashboard.putString("Alliance", alliance.toString());

    if (yButtonPress) {
      shooterRun = !shooterRun;
    }

    SmartDashboard.putBoolean("Shooter Run", shooterRun);

    if (shooterRun) {
      if (colorAccept) {
        shooter.set(TalonFXControlMode.PercentOutput, -.7);
      } else {
        shooter.set(TalonFXControlMode.PercentOutput, -.25);
      }

    } else {
      shooter.set(TalonFXControlMode.PercentOutput, 0);
    }

  }

  private void climb() {
    int pov = m_controller.getPOV();
    PovAngle povAngle;

    if (pov == 0) {
      povAngle = PovAngle.Up;
    } else if (pov == 90) {
      povAngle = PovAngle.Right;
    } else if (pov == 180) {
      povAngle = PovAngle.Down;
    } else if (pov == 270) {
      povAngle = PovAngle.Left;
    } else {
      povAngle = PovAngle.Bad;
    }

    switch (povAngle) {
      case Up:
        climbLeft.set(-.6);
        climbRight.set(.6);
        break;
      case Down:
        // if (!lsClimbLeft.get()) {
        climbLeft.set(.6);
        // }
        // if (!lsClimbRight.get()) {
        climbRight.set(-.6);
        // }
        if (lsClimbLeft.get() & lsClimbRight.get()) {
          // m_controller.setRumble(RumbleType.kLeftRumble, 1);
          // m_controller.setRumble(RumbleType.kRightRumble, 1);
        }
        break;
      case Left:
        climbCylinders.set(Value.kForward);
        break;
      case Right:
        climbCylinders.set(Value.kReverse);
        break;
      default:
        climbLeft.set(0);
        climbRight.set(0);
        climbCylinders.set(Value.kOff);
        // m_controller.setRumble(RumbleType.kLeftRumble, 0);
        // m_controller.setRumble(RumbleType.kRightRumble, 0);
        break;
    }

  }

  private enum PovAngle {
    Up,
    Down,
    Left,
    Right,
    Bad
  }

}
