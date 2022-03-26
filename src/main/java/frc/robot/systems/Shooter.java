package frc.robot.systems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class Shooter {
    private final CANSparkMax shooterAngle = new CANSparkMax(5, MotorType.kBrushless);
    private final TalonFX shooter = new TalonFX(3);
    private Color detectedColor;
    private final I2C.Port i2cPort = I2C.Port.kOnboard;

    private final ColorMatch m_colorMatcher = new ColorMatch();
    private final Color kBlueBall = new Color(0.17, 0.41, 0.41);
    private final Color kRedBall = new Color(0.51, 0.35, 0.15);
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
    private int proximity;
    
    public Shooter() {
        super();
        m_colorMatcher.addColorMatch(kBlueBall);
        m_colorMatcher.addColorMatch(kRedBall);
    }

    public void shoot(boolean shooterRun, boolean lsShooterHome, XboxController m_controller, Alliance alliance) {
        boolean rightBumper = m_controller.getRightBumper();
        boolean lefttBumper = m_controller.getLeftBumper();
        boolean yButtonPress = m_controller.getYButtonPressed();
        proximity = m_colorSensor.getProximity();
        double IR = m_colorSensor.getIR();
        detectedColor = m_colorSensor.getColor();
        SmartDashboard.putNumber("Proximity", proximity);
                SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putNumber("IR", IR);
        SmartDashboard.putBoolean("Right Bumper", rightBumper);
        SmartDashboard.putBoolean("Left Bumper", lefttBumper);
        SmartDashboard.putBoolean("Y Button", yButtonPress);

        if (rightBumper & lsShooterHome) {
            shooterAngle.set(-.1);
        } else if (rightBumper & lsShooterHome) {
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

}
