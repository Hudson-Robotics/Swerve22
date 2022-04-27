package frc.robot.systems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber {

    // #region Variables
    private final CANSparkMax climbLeft = new CANSparkMax(14, MotorType.kBrushless);
    private final CANSparkMax climbRight = new CANSparkMax(9, MotorType.kBrushless);

    private final Controller xboxCtrlr = Controller.getInstance();

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

        switch (xboxCtrlr.getPOV()) {
            case North:
            case NE:
            case NW:
                if (currentLeftPosition > -138) {
                    climbLeft.set(leftRamp.calculate(-.75));
                } else {
                    climbLeft.set(leftRamp.calculate(0));
                }
                if (currentRightPosition < 120) {
                    climbRight.set(rightRamp.calculate(.75));
                } else {
                    climbRight.set(rightRamp.calculate(0));
                }
                break;
            case South:
            case SE:
            case SW:
                if (currentLeftPosition < 0) {
                    climbLeft.set(leftRamp.calculate(.5));
                } else {
                    climbLeft.set(leftRamp.calculate(0));
                }
                if (currentRightPosition > 0) {
                    climbRight.set(rightRamp.calculate(-.5));
                } else {
                    climbRight.set(rightRamp.calculate(0));
                }
                break;
            case West:
                climbCylinders.set(Value.kForward);
                break;
            case East:
                climbCylinders.set(Value.kReverse);
                break;
            default:
                climbLeft.set(0);
                climbRight.set(0);
                climbCylinders.set(Value.kOff);
                break;
        }

        if (xboxCtrlr.getBackButton()) {
            climbLeft.set(leftRamp.calculate(.2));
        }
        if (xboxCtrlr.getStartButton()) {
            climbRight.set(rightRamp.calculate(-.2));
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

    private Climber() {
    }

    private final static Climber instance = new Climber();

    public static Climber getInstance() {
        return instance;
    }
}
