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
import frc.robot.subsystems.drivetrain.SwerveModule.DriveState;

public class Drivetrain extends SubsystemBase {

  private SwerveModule frontLeft = new SwerveModule(Constants.DrivetrainConstants.FrontLeft.kRotate, Constants.DrivetrainConstants.FrontLeft.kDrive); 
  private SwerveModule frontRight = new SwerveModule(Constants.DrivetrainConstants.FrontRight.kRotate, Constants.DrivetrainConstants.FrontRight.kDrive); 
  private SwerveModule backLeft = new SwerveModule(Constants.DrivetrainConstants.BackLeft.kRotate, Constants.DrivetrainConstants.BackLeft.kDrive); 
  private SwerveModule backRight = new SwerveModule(Constants.DrivetrainConstants.BackRight.kRotate, Constants.DrivetrainConstants.BackRight.kDrive); 

  private SwerveDrivePoseEstimator odometry; 

  private AHRS gyro = new AHRS(SPI.Port.kMXP);

  private DriveState driveState = DriveState.CLOSED_LOOP; 

  public Drivetrain() {
    
    this.gyro.zeroYaw();

    this.odometry = new SwerveDrivePoseEstimator(Constants.DrivetrainConstants.kDriveKinematics, gyro.getRotation2d(), getPositions(), new Pose2d()); 
  }

  @Override
  public void periodic() {
    this.odometry.update(gyro.getRotation2d(), getPositions()); 
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  
  public void swerveDrive(double xMetersPerSecond, double yMetersPerSecond, double rotationRadPerSecond) {
    swerveDrive(new ChassisSpeeds(
      xMetersPerSecond, 
      yMetersPerSecond, 
      rotationRadPerSecond
    ));
  }

  public void swerveDrive(ChassisSpeeds speeds) {
    Pose2d target = new Pose2d(speeds.vxMetersPerSecond * Constants.kLoopDuration, speeds.vyMetersPerSecond * Constants.kLoopDuration, Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * Constants.kLoopDuration));

    Twist2d twist = new Pose2d().log(target); 

    speeds = new ChassisSpeeds(twist.dx * Constants.kLoopDuration, twist.dy * Constants.kLoopDuration, twist.dtheta * Constants.kLoopDuration); 

    SwerveModuleState[] states = Constants.DrivetrainConstants.kDriveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
        speeds, 
        gyro.getRotation2d()
      )); 
      SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.DrivetrainConstants.kMaxAttainableModuleSpeedMetersPerSecond);
      swerveDrive(states);
  }

  public void swerveDrive(
    SwerveModuleState[] states) {
      this.frontLeft.updateState(states[0], driveState);
      this.frontRight.updateState(states[1], driveState);
      this.backLeft.updateState(states[2], driveState);
      this.backRight.updateState(states[3], driveState);
  }

  public void brake() {
    swerveDrive(Constants.DrivetrainConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds()));
  }

  public SwerveModulePosition[] getPositions() {
    return new SwerveModulePosition[] {
      frontLeft.getPosition(), 
      frontRight.getPosition(), 
      backLeft.getPosition(), 
      backRight.getPosition()
    }; 
  }
  
  public SwerveModuleState[] getStates() {
    return new SwerveModuleState[] {
      frontLeft.getCurrentState(), 
      frontRight.getCurrentState(), 
      backLeft.getCurrentState(), 
      backRight.getCurrentState()
    }; 
  }

  public ChassisSpeeds getChassisSpeeds() {
    return Constants.DrivetrainConstants.kDriveKinematics.toChassisSpeeds(getStates()); 
  }

  public Pose2d getPose() {
    return odometry.getEstimatedPosition(); 
  }

    // Returns the direction the robot is facing in degrees from -180 to 180 degrees.
    public double getHeading() {
      return gyro.getRotation2d().getDegrees();
  }

  public double getYaw() {
      return -gyro.getYaw(); 
  }

  // Returns the rate at which the robot is turning in degrees per second.
  public double getTurnRate() {
      return -gyro.getRate();
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(gyro.getRotation2d(), getPositions(), pose);
  }

  public void setDriveState(DriveState state) {
    this.driveState = state; 
  }
}
