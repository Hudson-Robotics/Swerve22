package frc.robot.systems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.systems.Controller.PovAngle;

public class Shooter {

    // #region Variables
    private final CANSparkMax shooterAngle = new CANSparkMax(5, MotorType.kBrushless);
    private final TalonFX shooter = new TalonFX(3);
    private final Controller xboxCtrlr = Controller.getInstance();
    private Color detectedColor = Color.kBlue;
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final DigitalInput lsShooterHome = new DigitalInput(0);
    private final ColorMatch m_colorMatcher = new ColorMatch();
    private final Color kBlueBall = new Color(0.17, 0.41, 0.41);
    private final Color kRedBall = new Color(0.51, 0.35, 0.15);
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
    private int proximity;
    private double IR;
    private String colorString;
    private boolean rightBumper;
    private boolean lefttBumper;
    private boolean shooterHome;
    private boolean shooterUp;
    private Alliance alliance;
    public boolean shooterRun;
    private ColorMatchResult match;
    private double currentPosition;
    // #endregion

    public void shoot() {
        PovAngle pov = xboxCtrlr.getPOV();
        if (pov == PovAngle.North || pov == PovAngle.NW || pov == PovAngle.NE) {
            Stop();
        }

        if (xboxCtrlr.getYButtonPressed()) {
            toggleMode();
        }

        currentPosition = shooterAngle.getEncoder().getPosition();
        shooterHome = lsShooterHome.get();
        shooterUp = currentPosition > 300;

        if (!shooterHome) {
            resetEncoder();
        }

        rightBumper = xboxCtrlr.getRightBumper();
        lefttBumper = xboxCtrlr.getLeftBumper();
        proximity = m_colorSensor.getProximity();
        IR = m_colorSensor.getIR();
        detectedColor = m_colorSensor.getColor();
        alliance = DriverStation.getAlliance();

        if (rightBumper & shooterHome) {
            shooterAngle.set(-.04);
        } else if (rightBumper & !shooterHome & !lefttBumper) {
            shooterAngle.set(0);
            xboxCtrlr.setRumble(1);
        } else if (lefttBumper & !rightBumper) {
            shooterAngle.set(.04);
        } else {
            shooterAngle.set(0);
            xboxCtrlr.stopRumble();
        }

        colorString = "Unknown";
        Boolean colorAccept = false;
        match = m_colorMatcher.matchClosestColor(detectedColor);

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

        if (shooterRun) {
            if (colorAccept) {
                Run(.4);
            } else {
                Run(.4);
            }
        } else {
            Stop();
        }

    }

    public void Run(double speed) {
        shooter.set(TalonFXControlMode.PercentOutput, -speed);
        // double rpmMax = 5000.0;

        /**
         * Convert 2000 RPM to units / 100ms.
         * 2048 Units/Rev * 2000 RPM / 600 100ms/min in either direction:
         * velocity setpoint is in units/100ms
         */
        // double targetVelocity_UnitsPer100ms = speed * rpmMax * 2048.0 / 600.0;

        /* 2000 RPM in either direction */
        // shooter.set(TalonFXControlMode.Velocity, targetVelocity_UnitsPer100ms);
    }

    public void Stop() {
        shooter.set(TalonFXControlMode.PercentOutput, 0);
        shooterRun = false;
    }

    public void resetEncoder() {
        shooterAngle.getEncoder().setPosition(0);
    }

    private void toggleMode() {
        shooterRun = !shooterRun;
    }

    public boolean getMode() {
        return shooterRun;
    }

    public void updateMeasurements() {
        SmartDashboard.putNumber("Proximity", proximity);
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putNumber("IR", IR);

        SmartDashboard.putBoolean("Right Bumper", rightBumper);
        SmartDashboard.putBoolean("Left Bumper", lefttBumper);

        // SmartDashboard.putNumber("Confidence", match.confidence);
        // SmartDashboard.putString("Alliance", alliance.toString());
        SmartDashboard.putString("Color Detected", colorString);

        SmartDashboard.putBoolean("Shooter Run", shooterRun);

        SmartDashboard.putNumber("Shooter Hood Position", currentPosition);
        SmartDashboard.putBoolean("Shooter Home", shooterHome);
        SmartDashboard.putBoolean("Shooter Max Up", shooterUp);

    }

    private Shooter() {
        m_colorMatcher.addColorMatch(kBlueBall);
        m_colorMatcher.addColorMatch(kRedBall);
        colorString = "Unknown";

        configPID();
    }

    private void configPID() {
        int ID = 0;
        int timeOutMS = 30;
        double feedForward = .0005;
        double proportional = .15;
        double integral = 0.001;
        double derivative = 5.0;

        /* Factory Default all hardware to prevent unexpected behaviour */
        shooter.configFactoryDefault();

        /* Config neutral deadband to be the smallest possible */
        shooter.configNeutralDeadband(0.001);

        /* Config sensor used for Primary PID [Velocity] */
        shooter.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                ID, timeOutMS);

        /* Config the peak and nominal outputs */
        shooter.configNominalOutputForward(0, timeOutMS);
        shooter.configNominalOutputReverse(0, timeOutMS);
        shooter.configPeakOutputForward(1, timeOutMS);
        shooter.configPeakOutputReverse(-1, timeOutMS);

        /* Config the Velocity closed loop gains in slot0 */
        shooter.config_kF(ID, feedForward, timeOutMS);
        shooter.config_kP(ID, proportional, timeOutMS);
        shooter.config_kI(ID, integral, timeOutMS);
        shooter.config_kD(ID, derivative, timeOutMS);

    }

    private static final Shooter instance = new Shooter();

    public static Shooter getInstance() {
        return instance;
    }
}
