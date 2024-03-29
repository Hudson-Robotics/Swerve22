package frc.robot.systems.Climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SmartMotion {
    // private static final int deviceID = 1;
    private CANSparkMax m_motor;
    private SparkMaxPIDController m_pidController;
    private RelativeEncoder m_encoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
    private String Name;

    public SmartMotion(int deviceID, String Name) {
        this.Name = Name;

        m_motor = new CANSparkMax(deviceID, MotorType.kBrushless);
        m_motor.restoreFactoryDefaults();

        // initialze PID controller and encoder objects
        m_pidController = m_motor.getPIDController();
        m_encoder = m_motor.getEncoder();

        // PID coefficients
        // kP = 5e-5;
        // kI = 1e-6;
        // kD = 0;
        // kIz = 0;
        // kFF = 0.000156;
        // kMaxOutput = 1;
        // kMinOutput = -1;
        // maxRPM = 5700;
        kP = 5e-5;
        kI = 1e-6;
        kD = 0;
        kIz = 0;
        kFF = 0.0005;
        kMaxOutput = 1;
        kMinOutput = -1;
        maxRPM = 5700;

        // Smart Motion Coefficients
        maxVel = 4000; // rpm
        maxAcc = 3000;

        // set PID coefficients
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);

        int smartMotionSlot = 0;
        m_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        m_pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        m_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        m_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    }

    public void UpdateMeasurements() {
        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);

        // display Smart Motion coefficients
        SmartDashboard.putNumber("Max Velocity", maxVel);
        SmartDashboard.putNumber("Min Velocity", minVel);
        SmartDashboard.putNumber("Max Acceleration", maxAcc);
        SmartDashboard.putNumber("Allowed Closed Loop Error", allowedErr);
        // SmartDashboard.putNumber("Set Position", 0);
        // SmartDashboard.putNumber("Set Velocity", 0);

        // button to toggle between velocity and smart motion modes
        // SmartDashboard.putBoolean("Mode", true);

    }

    public void UpdateLoopSettings() {
        // read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);
        double maxV = SmartDashboard.getNumber("Max Velocity", 0);
        double minV = SmartDashboard.getNumber("Min Velocity", 0);
        double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
        double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);
        // if PID coefficients on SmartDashboard have changed, write new values to
        // controller
        if ((p != kP)) {
            m_pidController.setP(p);
            kP = p;
        }
        if ((i != kI)) {
            m_pidController.setI(i);
            kI = i;
        }
        if ((d != kD)) {
            m_pidController.setD(d);
            kD = d;
        }
        if ((iz != kIz)) {
            m_pidController.setIZone(iz);
            kIz = iz;
        }
        if ((ff != kFF)) {
            m_pidController.setFF(ff);
            kFF = ff;
        }
        if ((max != kMaxOutput) || (min != kMinOutput)) {
            m_pidController.setOutputRange(min, max);
            kMinOutput = min;
            kMaxOutput = max;
        }
        if ((maxV != maxVel)) {
            m_pidController.setSmartMotionMaxVelocity(maxV, 0);
            maxVel = maxV;
        }
        if ((minV != minVel)) {
            m_pidController.setSmartMotionMinOutputVelocity(minV, 0);
            minVel = minV;
        }
        if ((maxA != maxAcc)) {
            m_pidController.setSmartMotionMaxAccel(maxA, 0);
            maxAcc = maxA;
        }
        if ((allE != allowedErr)) {
            m_pidController.setSmartMotionAllowedClosedLoopError(allE, 0);
            allowedErr = allE;
        }

    }

    public void Set(double setPoint, positionMode mode) {
        double processVariable;

        if (mode == positionMode.kVelocity) {
            // setPoint = SmartDashboard.getNumber("Set Velocity", 0);
            m_pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
            processVariable = m_encoder.getVelocity();
        } else {
            // setPoint = SmartDashboard.getNumber("Set Position", 0);
            /**
             * As with other PID modes, Smart Motion is set by calling the
             * setReference method on an existing pid object and setting
             * the control type to kSmartMotion
             */
            m_pidController.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
            processVariable = m_encoder.getPosition();
        }
        SmartDashboard.putNumber(Name + " Encoder", m_motor.getEncoder().getPosition());
        SmartDashboard.putNumber(Name + " Process Variable", processVariable);
        SmartDashboard.putNumber(Name + " Output", m_motor.getOutputCurrent());

    }

    public void resetEncoders() {
        m_motor.getEncoder().setPosition(0);
    }

    public double getEncoder() {
        return m_motor.getEncoder().getPosition();
    }

    public enum positionMode {
        kVelocity,
        kPosition
    }
}
