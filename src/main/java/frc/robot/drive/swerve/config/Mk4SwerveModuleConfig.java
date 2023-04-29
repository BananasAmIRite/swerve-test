package frc.robot.drive.swerve.config;

import edu.wpi.first.math.util.Units;
import frc.robot.drive.swerve.SwerveModuleConfig;

public class Mk4SwerveModuleConfig extends SwerveModuleConfig {
    private double wheelRadiusInches; 

    private int kDrivePort; 
    private int kSteerPort; 
    private int kEncoderPort; 

    public Mk4SwerveModuleConfig(double wheelRadiusInches, int kDrive, int kSteer, int kEncoder) {
        this.wheelRadiusInches = wheelRadiusInches; 

        this.kDrivePort = kDrive; 
        this.kSteerPort = kSteer; 
        this.kEncoderPort = kEncoder; 
    }

    @Override
    public double getWheelRadiusInches() {
        return this.wheelRadiusInches; 
    }

    @Override
    public double getDriveGearRatio() {
        return 6.75; 
    }

    @Override
    public double getSteerGearRatio() {
        return 12.8; 
    }

    @Override
    public int getDrivePort() {
        return this.kDrivePort; 
    }

    @Override
    public int getSteerPort() {
        return this.kSteerPort; 
    }

    @Override
    public int getEncoderPort() {
        return this.kEncoderPort; 
    }

    @Override
    public double getMaxAttainableSpeed() {
        return 5880.0 / 60.0 / getDriveGearRatio() * 2 * Units.inchesToMeters(getWheelRadiusInches()) * Math.PI; 
    }
}
