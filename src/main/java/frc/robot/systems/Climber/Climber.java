package frc.robot.systems.Climber;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.systems.Controller;
import frc.robot.systems.Climber.SmartMotion.positionMode;

public class Climber {

    // #region Variables
    // private final CANSparkMax climbLeft = new CANSparkMax(14,
    // MotorType.kBrushless);
    // private final CANSparkMax climbRight = new CANSparkMax(9,
    // MotorType.kBrushless);
    private final SmartMotion climbLeft = new SmartMotion(14, "Climb Left");
    private final SmartMotion climbRight = new SmartMotion(9, "Climb Right");

    private final Controller xboxCtrlr = Controller.getInstance();

    private final PneumaticHub PnueHub = new PneumaticHub(22);
    private final DoubleSolenoid climbCylinders = PnueHub.makeDoubleSolenoid(0, 1);

    private double currentLeftPosition;
    private double currentRightPosition;

    // private SlewRateLimiter leftRamp = new SlewRateLimiter(2);
    // private SlewRateLimiter rightRamp = new SlewRateLimiter(2);
    // #endregion

    public void climb() {
        // currentLeftPosition = climbLeft.getEncoder().getPosition();
        // currentRightPosition = climbRight.getEncoder().getPosition();
        // climbLeft.UpdateLoopSettings();
        // climbRight.UpdateLoopSettings();

        currentLeftPosition = climbLeft.getEncoder();
        currentRightPosition = climbRight.getEncoder();

        switch (xboxCtrlr.getPOV()) {
            case North:
            case NE:
            case NW:
                climbLeft.Set(-136, positionMode.kPosition);
                climbRight.Set(134, positionMode.kPosition);
                // if (currentLeftPosition > -138) {
                // climbLeft.set(leftRamp.calculate(-.75));
                // } else {
                // climbLeft.set(leftRamp.calculate(0));
                // }
                // if (currentRightPosition < 120) {
                // climbRight.set(rightRamp.calculate(.75));
                // } else {
                // climbRight.set(rightRamp.calculate(0));
                // }
                break;
            case South:
            case SE:
            case SW:
                climbLeft.Set(0, positionMode.kPosition);
                climbRight.Set(0, positionMode.kPosition);
                // if (currentLeftPosition < 0) {
                // climbLeft.set(leftRamp.calculate(.5));
                // } else {
                // climbLeft.set(leftRamp.calculate(0));
                // }
                // if (currentRightPosition > 0) {
                // climbRight.set(rightRamp.calculate(-.5));
                // } else {
                // climbRight.set(rightRamp.calculate(0));
                // }
                break;
            case West:
                climbCylinders.set(Value.kForward);
                break;
            case East:
                climbCylinders.set(Value.kReverse);
                break;
            default:
                // climbLeft.set(0);
                // climbRight.set(0);
                // climbLeft.Set(0, positionMode.kVelocity);
                // climbRight.Set(0, positionMode.kVelocity);
                climbLeft.Set(currentLeftPosition, positionMode.kPosition);
                climbRight.Set(currentRightPosition, positionMode.kPosition);
                climbCylinders.set(Value.kOff);
                break;
        }

        if (xboxCtrlr.getBackButton()) {
            // climbLeft.set(leftRamp.calculate(.2));
            // climbLeft.Set(2000, positionMode.kVelocity);
            climbLeft.Set(currentLeftPosition + 6, positionMode.kPosition);
        }
        if (xboxCtrlr.getStartButton()) {
            // climbRight.set(rightRamp.calculate(-.2));
            // climbRight.Set(-2000, positionMode.kVelocity);
            climbRight.Set(currentRightPosition - 6, positionMode.kPosition);
        }

    }

    public void resetEncoders() {
        climbLeft.resetEncoders();
        climbRight.resetEncoders();
    }

    public void updateMeasurements() {
        climbLeft.UpdateMeasurements();
        climbRight.UpdateMeasurements();
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
