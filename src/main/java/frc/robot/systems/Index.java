package frc.robot.systems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.I2C;

public class Index {
    private final CANSparkMax indexTop = new CANSparkMax(10, MotorType.kBrushless);
    private final CANSparkMax indexBottom = new CANSparkMax(11, MotorType.kBrushless);
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
    private int proximity = 0;

    private static final Index instance = new Index();

    private Index() {
    }

    public static Index getInstance() {
        return instance;
    }

    public void index(XboxController m_controller, boolean shooterRun) {
        boolean aButton = m_controller.getAButton();
        boolean bButton = m_controller.getBButton();
        boolean xButton = m_controller.getXButton();
        proximity = m_colorSensor.getProximity();
        boolean prox;

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
