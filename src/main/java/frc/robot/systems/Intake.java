package frc.robot.systems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake {
    private final TalonSRX intake = new TalonSRX(13);

    private final XboxController xboxCtrlr = new XboxController(0);

    private final PneumaticHub PnueHub = new PneumaticHub(22);
    private final DoubleSolenoid intLeftCylinders = PnueHub.makeDoubleSolenoid(2, 3);
    private final DoubleSolenoid intRightCylinders = PnueHub.makeDoubleSolenoid(6, 7);
    private boolean leftTriggerPressed;
    private boolean rightTriggerPressed;
    private double leftTrigger;
    private double rightTrigger;

    public void intake(boolean shooterRun) {
        leftTrigger = xboxCtrlr.getLeftTriggerAxis();
        rightTrigger = xboxCtrlr.getRightTriggerAxis();

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

        if (shooterRun && !rightTriggerPressed) {
            Up();
        } else if (shooterRun && rightTriggerPressed) {
            Down();
        } else {
            Stop();
        }
    }

    private void Up() {
        intake.set(TalonSRXControlMode.PercentOutput, -.7);
        intLeftCylinders.set(Value.kForward);
        intRightCylinders.set(Value.kForward);
    }

    private void Down() {
        intake.set(TalonSRXControlMode.PercentOutput, -.7);
        intLeftCylinders.set(Value.kReverse);
        intRightCylinders.set(Value.kReverse);

    }

    private void Stop() {
        intake.set(TalonSRXControlMode.PercentOutput, 0);
        intLeftCylinders.set(Value.kOff);
        intRightCylinders.set(Value.kOff);

    }

    public void updateMeasurements() {
        SmartDashboard.putBoolean("Left Trigger Bool", leftTriggerPressed);
        SmartDashboard.putNumber("Left Trigger Position", leftTrigger);
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
