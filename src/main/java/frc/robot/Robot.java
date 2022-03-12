package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.ColorSensorV3;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

public class Robot extends TimedRobot {
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  private final XboxController m_controller = new XboxController(0);
  private final Drivetrain m_swerve = new Drivetrain();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(2);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(2);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(2);

  private final CANSparkMax climbLeft = new CANSparkMax(14, MotorType.kBrushless);
  private final CANSparkMax climbRight = new CANSparkMax(9, MotorType.kBrushless);

  private final CANSparkMax indexTop = new CANSparkMax(11, MotorType.kBrushless);
  private final CANSparkMax indexBottom = new CANSparkMax(10, MotorType.kBrushless);

  private final CANSparkMax shooterAngle = new CANSparkMax(5, MotorType.kBrushless);
  private final TalonFX shooter = new TalonFX(3);

  private final TalonSRX intake = new TalonSRX(13);

  private final PneumaticHub PnueHub = new PneumaticHub(22);
  private final DoubleSolenoid climbCylinders = PnueHub.makeDoubleSolenoid(0, 1);
  private final DoubleSolenoid intLeftCylinders = PnueHub.makeDoubleSolenoid(2, 3);
  private final DoubleSolenoid intRightCylinders = PnueHub.makeDoubleSolenoid(4, 5);
  private final double ptHigh = PnueHub.getPressure(0);
  private final double ptWork = PnueHub.getPressure(1);
  private final double compCurrent = PnueHub.getCompressorCurrent();

  private final DigitalInput lsShooterHome = new DigitalInput(0);
  private final DigitalInput lsClimbLeft = new DigitalInput(1);
  private final DigitalInput lsClimbRight = new DigitalInput(2);

  private int proximity;

  @Override
  public void robotInit() {
  }

  @Override
  public void robotPeriodic() {
    proximity = m_colorSensor.getProximity();
    double IR = m_colorSensor.getIR();
    Color detectedColor = m_colorSensor.getColor();

    SmartDashboard.putNumber("Proximity", proximity);

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", IR);

    m_swerve.updateOdometry();
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
    driveWithJoystick(false);
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    if (m_controller.getLeftBumperPressed() && m_controller.getRightBumperPressed()) {
      m_swerve.Reset();
    }

    driveWithJoystick(true);
    intake();
    index();
    shoot();

  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  private void driveWithJoystick(boolean fieldRelative) {
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

    if (leftTriggerPressed || rightTriggerPressed) {
      intake.set(ControlMode.PercentOutput, .3);
    } else {
      intake.set(ControlMode.PercentOutput, 0);
      intLeftCylinders.set(Value.kOff);
      intRightCylinders.set(Value.kOff);
    }

    if (leftTriggerPressed) {
      intLeftCylinders.set(Value.kForward);
      intRightCylinders.set(Value.kForward);
    }
    if (rightTriggerPressed) {
      intLeftCylinders.set(Value.kReverse);
      intRightCylinders.set(Value.kReverse);
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

    if (!aButton & xButton) {
      indexTop.set(-.3);
      indexBottom.set(-.3);
    } else if (aButton & !prox) {
      indexTop.set(.3);
      indexBottom.set(.3);
    } else if (bButton) {
      indexTop.set(.3);
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

    if (rightBumper & !lsShooterHome.get()) {
      shooterAngle.set(-.1);
    } else if (lefttBumper) {
      shooterAngle.set(.1);
    } else {
      shooterAngle.set(0);
    }

    if (yButtonPress) {

    }
  }
}
