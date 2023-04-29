package frc.robot.drive.swerve;

import frc.robot.drive.WheelConfig;

public abstract class SwerveModuleConfig extends WheelConfig {
    public SwerveModuleConfig(int port) {
        super(port);
    }

    public int getDriveCurrentLimit() {
        return 60; 
    }

    // turn 
    
    public abstract double getSteerGearRatio();

    /**
     * Steer position conversion in Degrees per Rotation
     */
    public double getSteerPositionConversion() {
        return 360.0 / getSteerGearRatio(); 
    }

    /**
     * Steer position conversion in Degrees per Second per RPM
     */
    public double getSteerVelocityConversion() {
        return getSteerPositionConversion() / 60; 
    }

    public int getSteerCurrentLimit() {
        return 30; 
    }

    public abstract int getDrivePort(); 

    public abstract int getSteerPort(); 

    public abstract int getEncoderPort();
}