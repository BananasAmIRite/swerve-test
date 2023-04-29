package frc.robot.drive.swerve;

import java.util.List;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.drive.DriveSignal;
import frc.robot.drive.DrivetrainBase;
import frc.robot.drive.swerve.SwerveModule.DriveState;

public class SwerveDrivetrain extends DrivetrainBase {
    private final SwerveDrivetrainConfig config; 

    private SwerveModule[] modules; 
    private SwerveDriveKinematics kinematics; 

    private DriveState driveState = DriveState.CLOSED_LOOP; 

    private final SwerveDrivePoseEstimator odometry;

    public SwerveDrivetrain(SwerveDrivetrainConfig config) {
        this.config = config;

        setupModules();
        setupKinematics();

        this.gyro.zeroYaw();

        this.odometry = new SwerveDrivePoseEstimator(Constants.DrivetrainConstants.kDriveKinematics, gyro.getRotation2d(), getPositions(), new Pose2d()); 
    }

    private void setupModules() {
        List<SwerveModuleConfig> moduleConfigs = config.getModuleConfigs(); 

        this.modules = new SwerveModule[moduleConfigs.size()];
        for (int i = 0; i < moduleConfigs.size(); i++) {
            SwerveModuleConfig modConfig = moduleConfigs.get(i); 
            this.modules[i] = new SwerveModule(modConfig); 
        }
    }

    private void setupKinematics() {
        List<Translation2d> moduleTranslations = config.getModuleTranslations(); 

        Translation2d[] translations = new Translation2d[moduleTranslations.size()];

        for (int i = 0; i < moduleTranslations.size(); i++) {
            translations[i] = moduleTranslations.get(i); 
        }

        this.kinematics = new SwerveDriveKinematics(translations); 
    }

    @Override
    public void drive(DriveSignal signal) {
        ChassisSpeeds speeds = signal.getSpeeds(); 
        if (signal.isFieldRelative()) speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, gyro.getRotation2d()); 

        drive(speeds);
    }

    public void drive(ChassisSpeeds speeds) {
        Pose2d target = new Pose2d(speeds.vxMetersPerSecond * Constants.kLoopDuration, speeds.vyMetersPerSecond * Constants.kLoopDuration, Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * Constants.kLoopDuration));
    
        Twist2d twist = new Pose2d().log(target); 
    
        speeds = new ChassisSpeeds(twist.dx * Constants.kLoopDuration, twist.dy * Constants.kLoopDuration, twist.dtheta * Constants.kLoopDuration); 
    
        SwerveModuleState[] states = Constants.DrivetrainConstants.kDriveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds, 
            gyro.getRotation2d()
          )); 
        SwerveDriveKinematics.desaturateWheelSpeeds(states, config.getMaxModuleSpeed());
        drive(states);
    }

    public void drive(SwerveModuleState[] states) {
        for (int i = 0; i < states.length; i++) {
            this.modules[i].updateState(states[i], driveState);
        }
    }

    
  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[this.modules.length]; 
    for (int i = 0; i < this.modules.length; i++) states[i] = this.modules[i].getPosition(); 
    return states;
  }
  
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[this.modules.length]; 
    for (int i = 0; i < this.modules.length; i++) states[i] = this.modules[i].getCurrentState(); 
    return states;
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(gyro.getRotation2d(), getPositions(), pose);
  }

  public void setDriveState(DriveState state) {
    this.driveState = state; 
  }

  public SwerveDriveKinematics getKinematics() {
    return this.kinematics; 
  }

  public Pose2d getPose() {
        return this.odometry.getEstimatedPosition();
  }

    @Override
    public void periodic() {

    }
}