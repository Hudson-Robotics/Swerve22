package frc.robot.systems.DriveTrain;


import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TalonSwerve {
        private static final double kWheelRadius = 0.0508;
        private static final int kLampreyEncoderResolution = 1024;

        private TalonSRX turningMotor;
        private TalonFX driveMotor;

        private String Name;

        public TalonSwerve(int driveMotorCanbusAddress, int turningMotorCanbusAddress, String Name) {
                turningMotor = new TalonSRX(turningMotorCanbusAddress);
                driveMotor = new TalonFX(driveMotorCanbusAddress);
                this.Name = Name;
        }

        public SwerveModuleState getState() {
                return new SwerveModuleState(driveMotor.getSelectedSensorVelocity() / 2 * Math.PI * kWheelRadius,
                                new Rotation2d(turningMotor.getSelectedSensorPosition() / kLampreyEncoderResolution * 2
                                                * Math.PI));
        }

        public void setDesiredState(SwerveModuleState desiredState) {

                // Optomize avoids spinning further than 90 degrees
                SwerveModuleState state = SwerveModuleState.optimize(desiredState,
                                new Rotation2d(turningMotor.getSelectedSensorPosition() / kLampreyEncoderResolution * 2
                                                * Math.PI));
                double turnRadians = turningMotor.getSelectedSensorPosition() / kLampreyEncoderResolution * 2 * Math.PI;

                turningMotor.set(TalonSRXControlMode.PercentOutput,
                                (state.angle.getRadians() - turnRadians) / 2);

                driveMotor.set(TalonFXControlMode.PercentOutput, state.speedMetersPerSecond / 3);

                SmartDashboard.putString(Name + "desiredState", desiredState.angle.toString());
        }
}

