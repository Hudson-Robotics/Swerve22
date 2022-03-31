package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.systems.Climber;
import frc.robot.systems.Index;
import frc.robot.systems.Intake;
import frc.robot.systems.LimeLight;
import frc.robot.systems.PneuHub;
import frc.robot.systems.Shooter;
import frc.robot.systems.DriveTrain.Drivetrain;

public class Robot extends TimedRobot {

  private final LimeLight limeLight = LimeLight.getInstance();
  private final Shooter shooter = Shooter.getInstance();
  private final Climber climber = Climber.getInstance();
  private final Index index = Index.getInstance();
  private final Intake intake = Intake.getInstance();
  private final Drivetrain swerve = Drivetrain.getInstance();
  private final PneuHub pneuHub = PneuHub.getInstance();

  @Override
  public void robotPeriodic() {
    limeLight.updateMeasurements();
    pneuHub.updateMeasurements();
    swerve.updateOdometry();
    climber.updateMeasurements();
    intake.updateMeasurements();
    shooter.updateMeasurements();
  }

  private Timer timer = new Timer();

  @Override
  public void autonomousInit() {
    timer.reset();
    timer.start();
  }

  @Override
  public void autonomousPeriodic() {

    if (timer.get() < 2) {
      shooter.Run(.5);
    } else if (timer.get() < 5) {
      index.Forward(.3);
    } else if (timer.get() < 8) {
      swerve.Reverse(.3 * Drivetrain.kMaxSpeed);
    } else {
      shooter.Stop();
      index.Stop();
      swerve.Stop();
    }
  }

  @Override
  public void teleopInit() {
    climber.resetEncoders();
    shooter.resetEncoder();
  }

  @Override
  public void teleopPeriodic() {
    swerve.Drive(true);
    intake.intake(shooter.getMode());
    index.index(shooter.getMode());
    shooter.shoot();
    climber.climb();
  }
}
