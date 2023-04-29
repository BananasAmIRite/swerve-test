package frc.robot.drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class DrivetrainBase extends SubsystemBase {
    protected AHRS gyro = new AHRS(SPI.Port.kMXP);

    public DrivetrainBase() {

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

    public AHRS getGyro() {
        return gyro; 
    }

    public abstract void drive(DriveSignal signal); 
}
