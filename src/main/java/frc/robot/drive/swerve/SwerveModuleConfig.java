package frc.robot.drive.swerve;

import edu.wpi.first.math.util.Units;

public abstract class SwerveModuleConfig {
    public abstract double getWheelRadiusInches();

    // drive
    public abstract double getDriveGearRatio();

    /**
     * Drive position conversion in Meters per Rotation
     */
    public double getDrivePositionConversion() {
        return Units.inchesToMeters(2 * Math.PI * getWheelRadiusInches() / getDriveGearRatio());
    }

    /**
     * Drive velocity conversion in Meters per Second per RPM
     */
    public double getDriveVelocityConversion() {
        return getDrivePositionConversion() / 60; 
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

    public abstract double getMaxAttainableSpeed(); 
}