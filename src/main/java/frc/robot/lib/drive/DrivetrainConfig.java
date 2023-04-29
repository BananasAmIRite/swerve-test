package frc.robot.lib.drive;

public abstract class DrivetrainConfig {
    public abstract double getTrackWidthMeters(); 
    public abstract double getWheelBaseMeters();

    public abstract double getMaxDriveSpeed();

    public double getMaxRotationSpeed() {
        return getMaxDriveSpeed() / Math.hypot(getTrackWidthMeters() / 2.0, getWheelBaseMeters() / 2.0);
    }
}
