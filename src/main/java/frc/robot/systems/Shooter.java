package frc.robot.systems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class Shooter {

    // #region Variables
    private final CANSparkMax shooterAngle = new CANSparkMax(5, MotorType.kBrushless);
    private final TalonFX shooter = new TalonFX(3);
    private final XboxController xboxCtrlr = new XboxController(0);
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
    private double currentPosition; // #endregion

    public void shoot() {
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
            shooterAngle.set(-.08);
        } else if (rightBumper & !shooterHome & !lefttBumper) {
            shooterAngle.set(0);
            xboxCtrlr.setRumble(RumbleType.kLeftRumble, 1);
            xboxCtrlr.setRumble(RumbleType.kRightRumble, 1);
        } else if (lefttBumper & !rightBumper) {
            shooterAngle.set(.08);
        } else {
            shooterAngle.set(0);
            xboxCtrlr.setRumble(RumbleType.kLeftRumble, 0);
            xboxCtrlr.setRumble(RumbleType.kRightRumble, 0);
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
                Run(.47);
            } else {
                Run(.3);
                }
        } else {
            Stop();
        }

    }

    public void Run(double speed) {
        shooter.set(TalonFXControlMode.PercentOutput, -speed);
    }

    public void Stop() {
        shooter.set(TalonFXControlMode.PercentOutput, 0);
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
    }

    private static final Shooter instance = new Shooter();

    public static Shooter getInstance() {
        return instance;
    }
}
