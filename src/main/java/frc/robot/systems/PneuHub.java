package frc.robot.systems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PneuHub {
    private static final PneumaticHub pneumaticHub = new PneumaticHub(22);
    private static final PneuHub instance = new PneuHub();

    private double ptHigh;
    private double ptWork;
    private double compCurrent;

    private PneuHub() {
    }

    public static PneuHub getInstance() {
        return instance;
    }

    public void updateMeasurements() {
        ptHigh = pneumaticHub.getPressure(0);
        ptWork = pneumaticHub.getPressure(1);
        compCurrent = pneumaticHub.getCompressorCurrent();

        SmartDashboard.putNumber("PT High", ptHigh);
        SmartDashboard.putNumber("PT Work", ptWork);
        SmartDashboard.putNumber("Air Comp Amps", compCurrent);
    }
}
