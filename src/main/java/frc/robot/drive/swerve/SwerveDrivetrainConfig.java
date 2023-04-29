package frc.robot.drive.swerve;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.drive.DrivetrainConfig;

public class SwerveDrivetrainConfig extends DrivetrainConfig {

    private double trackWidth; 
    private double driveBase; 

    private List<SwerveModuleLocation> locations = new ArrayList<>(); 

    private double maxModuleSpeed = 0; 

    public SwerveDrivetrainConfig(double trackWidth, double driveBase) {
        this.trackWidth = trackWidth; 
        this.driveBase = driveBase; 
    }

    public SwerveDrivetrainConfig addSwerveModule(Translation2d translation, SwerveModuleConfig config) {
        this.locations.add(new SwerveModuleLocation(translation, config));
        this.maxModuleSpeed = Collections.min(locations, (a, b) -> (int) (a.config.getMaxAttainableSpeed() - b.config.getMaxAttainableSpeed())).config.getMaxAttainableSpeed(); 
        return this; 
    }

    @Override
    public double getTrackWidthMeters() {
        return trackWidth; 
    }

    @Override
    public double getWheelBaseMeters() {
        return driveBase;
    }

    public double getMaxModuleSpeed() {
        return maxModuleSpeed; 
    }

    public double getMaxDriveSpeed() {
        return getMaxModuleSpeed(); 
    }

    public double getMaxRotationSpeed() {
        return getMaxDriveSpeed() / Math.hypot(getTrackWidthMeters() / 2.0, getWheelBaseMeters() / 2.0); 
    }

    private static class SwerveModuleLocation {
        Translation2d translation; 
        SwerveModuleConfig config; 

        public SwerveModuleLocation(Translation2d translation, SwerveModuleConfig config) {
            this.translation = translation; 
            this.config = config; 
        }
    }
}
