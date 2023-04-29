package frc.robot.lib.drive.swerve;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.lib.drive.DrivetrainConfig;

public class SwerveDrivetrainConfig extends DrivetrainConfig {

    private final double trackWidth;
    private final double wheelBase;

    private final List<SwerveModuleLocation> locations = new ArrayList<>();

    private SwerveDriveKinematics kinematics;

    private double maxModuleSpeed = 0; 

    public SwerveDrivetrainConfig(double trackWidth, double driveBase) {
        this.trackWidth = trackWidth; 
        this.wheelBase = driveBase;
    }

    public void addSwerveModule(Translation2d translation, SwerveModuleConfig config) {
        this.locations.add(new SwerveModuleLocation(translation, config));
        this.maxModuleSpeed = Collections.min(locations, (a, b) -> (int) (a.config.getMaxAttainableSpeed() - b.config.getMaxAttainableSpeed())).config.getMaxAttainableSpeed();
        this.kinematics = generateKinematics();
    }

    private SwerveDriveKinematics generateKinematics() {
        List<Translation2d> moduleTranslations = getModuleTranslations();

        Translation2d[] translations = new Translation2d[moduleTranslations.size()];

        for (int i = 0; i < moduleTranslations.size(); i++) {
            translations[i] = moduleTranslations.get(i);
        }

        return new SwerveDriveKinematics(translations);
    }

    @Override
    public double getTrackWidthMeters() {
        return trackWidth; 
    }

    @Override
    public double getWheelBaseMeters() {
        return wheelBase;
    }

    public double getMaxModuleSpeed() {
        return maxModuleSpeed; 
    }

    @Override
    public double getMaxDriveSpeed() {
        return getMaxModuleSpeed(); 
    }

    public SwerveDriveKinematics getKinematics() {
        return this.kinematics; 
    }

    public List<SwerveModuleConfig> getModuleConfigs() {
        return locations.stream().map(e -> e.config).collect(Collectors.toList());
    }

    public List<Translation2d> getModuleTranslations() {
        return locations.stream().map(e -> e.translation).collect(Collectors.toList());
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