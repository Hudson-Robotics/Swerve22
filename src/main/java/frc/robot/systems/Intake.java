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
    private final PneumaticHub PnueHub = new PneumaticHub(22);
    private final DoubleSolenoid intLeftCylinders = PnueHub.makeDoubleSolenoid(2, 3);
    private final DoubleSolenoid intRightCylinders = PnueHub.makeDoubleSolenoid(6, 7);
   
    public void intake(XboxController m_controller, boolean shooterRun) {
        double leftTrigger = m_controller.getLeftTriggerAxis();
        double rightTrigger = m_controller.getRightTriggerAxis();
        boolean leftTriggerPressed;
        boolean rightTriggerPressed;

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

        SmartDashboard.putBoolean("Left Trigger Bool", leftTriggerPressed);
        SmartDashboard.putNumber("Left Trigger Position", leftTrigger);
        SmartDashboard.putBoolean("Right Trigger Bool", rightTriggerPressed);
        SmartDashboard.putNumber("Right Trigger Position", rightTrigger);

        if (shooterRun) {
            intake.set(TalonSRXControlMode.PercentOutput, -.7);
            intLeftCylinders.set(Value.kReverse);
            intRightCylinders.set(Value.kReverse);
        } else if (rightTriggerPressed) {
            intLeftCylinders.set(Value.kForward);
            intRightCylinders.set(Value.kForward);
        } else {
            intake.set(TalonSRXControlMode.PercentOutput, 0);
            intLeftCylinders.set(Value.kOff);
            intRightCylinders.set(Value.kOff);
        }
    }
}
