package frc.robot.systems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Index {
    private final CANSparkMax indexTop = new CANSparkMax(10, MotorType.kBrushless);
    private final CANSparkMax indexBottom = new CANSparkMax(11, MotorType.kBrushless);

    public void index(XboxController m_controller, boolean shooterRun) {
        boolean aButton = m_controller.getAButton();
        boolean bButton = m_controller.getBButton();
        boolean xButton = m_controller.getXButton();
        boolean prox;
        int proximity=100;

        if (proximity >= 200) {
            prox = true;
        } else {
            prox = false;
        }

        SmartDashboard.putBoolean("A Button", aButton);
        SmartDashboard.putBoolean("B Button", bButton);
        SmartDashboard.putBoolean("X Button", xButton);
        SmartDashboard.putBoolean("Prox", prox);

        if (!aButton & xButton) {
            indexTop.set(.3);
            indexBottom.set(-.3);
        } else if (aButton & !prox) {
            indexTop.set(-.3);
            indexBottom.set(.3);
        } else if (bButton & shooterRun) {
            indexTop.set(-.3);
            indexBottom.set(.3);
        } else {
            indexTop.set(0);
            indexBottom.set(0);
        }

    }
}
