package frc.robot.systems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber {

    // #region Variables
    private final CANSparkMax climbLeft = new CANSparkMax(14, MotorType.kBrushless);
    private final CANSparkMax climbRight = new CANSparkMax(9, MotorType.kBrushless);

    private final XboxController xboxCtrlr = new XboxController(0);

    private final PneumaticHub PnueHub = new PneumaticHub(22);
    private final DoubleSolenoid climbCylinders = PnueHub.makeDoubleSolenoid(0, 1);

    private double currentLeftPosition;
    private double currentRightPosition;

    private SlewRateLimiter leftRamp = new SlewRateLimiter(2);
    private SlewRateLimiter rightRamp = new SlewRateLimiter(2);
    // #endregion

    public void climb() {
        currentLeftPosition = climbLeft.getEncoder().getPosition();
        currentRightPosition = climbRight.getEncoder().getPosition();

        int pov = xboxCtrlr.getPOV();
        PovAngle povAngle;

        if (pov == 0 || pov == 45 || pov == 135) {
            povAngle = PovAngle.Up;
        } else if (pov == 90) {
            povAngle = PovAngle.Right;
        } else if (pov == 180) {
            povAngle = PovAngle.Down;
        } else if (pov == 270 || pov ==  225 || pov == 315) {
            povAngle = PovAngle.Left;
        } else {
            povAngle = PovAngle.Bad;
        }

        switch (povAngle) {
            case Up:
                if (currentLeftPosition > -625) {
                    climbLeft.set(leftRamp.calculate(-1));
                } else {
                    climbLeft.set(leftRamp.calculate(0));
                }
                if (currentRightPosition < 647) {
                    climbRight.set(rightRamp.calculate(1));
                } else {
                    climbRight.set(rightRamp.calculate(0));
                }
                break;
            case Down:
                if (currentLeftPosition < 0) {
                    climbLeft.set(leftRamp.calculate(.825));
                } else {
                    climbLeft.set(leftRamp.calculate(0));
                }
                if (currentRightPosition > 0) {
                    climbRight.set(rightRamp.calculate(-.825));
                } else {
                    climbRight.set(rightRamp.calculate(0));
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
                break;
        }

        if (xboxCtrlr.getBackButton()) {
            climbLeft.set(leftRamp.calculate(.4));
        }
        if (xboxCtrlr.getStartButton()) {
            climbRight.set(rightRamp.calculate(-.4));
        }
    }

    public void resetEncoders() {
        climbLeft.getEncoder().setPosition(0);
        climbRight.getEncoder().setPosition(0);
    }

    public void updateMeasurements() {
        SmartDashboard.putNumber("Encoder Left Arm", currentLeftPosition);
        SmartDashboard.putNumber("Encoder Right Arm", currentRightPosition);
    }

    private enum PovAngle {
        Up,
        Down,
        Left,
        Right,
        Bad
    }

    private Climber() {
    }

    private final static Climber instance = new Climber();

    public static Climber getInstance() {
        return instance;
    }
}
