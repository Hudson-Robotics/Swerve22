package frc.robot.systems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Climber {
    private final CANSparkMax climbLeft = new CANSparkMax(14, MotorType.kBrushless);
    private final CANSparkMax climbRight = new CANSparkMax(9, MotorType.kBrushless);

    private final PneumaticHub PnueHub = new PneumaticHub(22);
    private final DoubleSolenoid climbCylinders = PnueHub.makeDoubleSolenoid(0, 1);

    private final static Climber instance = new Climber();

    private Climber() {
    }

    public static Climber getInstance() {
        return instance;
    }

    public void climb(XboxController m_controller, boolean lsClimbLeft, boolean lsClimbRight) {
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
                if (lsClimbLeft & lsClimbRight) {
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
