package frc.robot.systems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake {

    //#region Variables
    private final TalonSRX intake = new TalonSRX(13);

    private final XboxController xboxCtrlr = new XboxController(0);

    private final PneumaticHub PnueHub = new PneumaticHub(22);
    private final DoubleSolenoid intLeftCylinders = PnueHub.makeDoubleSolenoid(2, 3);
    private final DoubleSolenoid intRightCylinders = PnueHub.makeDoubleSolenoid(6, 7);
    private boolean rightTriggerPressed;
    private double rightTrigger;
    private boolean aButton;
    private boolean bButton;
    //#endregion

    public void intake(boolean shooterRun) {
        rightTrigger = xboxCtrlr.getRightTriggerAxis();
        aButton = xboxCtrlr.getAButton();
        bButton = xboxCtrlr.getBButton();

        if (rightTrigger > .5) {
            rightTriggerPressed = true;
        } else {
            rightTriggerPressed = false;
        }

        if (shooterRun && !rightTriggerPressed) {
            Up();
        } else if (shooterRun && rightTriggerPressed) {
            Down();
        } else {
            Off();
        }

        if (aButton || bButton || rightTriggerPressed) {
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

    public void updateMeasurements() {
        SmartDashboard.putBoolean("Right Trigger Bool", rightTriggerPressed);
        SmartDashboard.putNumber("Right Trigger Position", rightTrigger);
    }

    private static final Intake instance = new Intake();

    private Intake() {
    }

    public static Intake getInstance() {
        return instance;
    }
}
