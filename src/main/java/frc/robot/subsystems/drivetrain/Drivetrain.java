package frc.robot.subsystems.drivetrain;

import frc.robot.lib.drive.swerve.SwerveDrivetrain;
import frc.robot.lib.drive.swerve.SwerveDrivetrainConfig;

public class Drivetrain extends SwerveDrivetrain {

  public Drivetrain(SwerveDrivetrainConfig config) {
    super(config);
  }

  @Override
  public void periodic() {
    super.periodic();
  }

  @Override
  public void simulationPeriodic() {}
}
