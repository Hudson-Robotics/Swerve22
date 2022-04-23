package frc.robot.systems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Intake {

    // #region Variables
    private final TalonSRX intake = new TalonSRX(13);

    private final Controller xboxCtrlr = Controller.getInstance();

    private final PneumaticHub PnueHub = new PneumaticHub(22);
    private final DoubleSolenoid intLeftCylinders = PnueHub.makeDoubleSolenoid(2, 3);
    private final DoubleSolenoid intRightCylinders = PnueHub.makeDoubleSolenoid(6, 7);
    private boolean rightTrigger;
    private boolean aButton;
    private boolean bButton;
    // #endregion

    public void intake(boolean shooterRun) {
        rightTrigger = xboxCtrlr.getRightTriggerPress();
        aButton = xboxCtrlr.getAButton();
        bButton = xboxCtrlr.getBButton();

        if (shooterRun && !rightTrigger) {
            Up();
        } else if (shooterRun && rightTrigger) {
            Down();
        } else {
            Off();
        }

        if (aButton || bButton || rightTrigger) {
            Run();
        } else {
            Stop();
        }
    }

    private void Up() {
        intLeftCylinders.set(Value.kForward);
        intRightCylinders.set(Value.kForward);
    }

    private void Down() {
        intLeftCylinders.set(Value.kReverse);
        intRightCylinders.set(Value.kReverse);
    }

    private void Off() {
        intLeftCylinders.set(Value.kOff);
        intRightCylinders.set(Value.kOff);
    }

    private void Stop() {
        intake.set(TalonSRXControlMode.PercentOutput, 0);
    }

    private void Run() {
        intake.set(TalonSRXControlMode.PercentOutput, -.7);
    }

    private static final Intake instance = new Intake();

    private Intake() {
    }

    public static Intake getInstance() {
        return instance;
    }
}
