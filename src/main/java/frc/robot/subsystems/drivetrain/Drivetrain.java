package frc.robot.subsystems.drivetrain;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.drive.swerve.SwerveDrivetrain;
import frc.robot.lib.drive.swerve.SwerveDrivetrainConfig;
import frc.robot.lib.drive.swerve.SwerveModule;
import frc.robot.lib.drive.swerve.SwerveModule.DriveState;

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
