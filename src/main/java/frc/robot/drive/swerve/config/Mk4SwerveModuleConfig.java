package frc.robot.drive.swerve.config;

import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.util.Units;
import frc.robot.drive.swerve.SwerveModuleConfig;

public class Mk4SwerveModuleConfig extends SwerveModuleConfig {
    private final double wheelRadiusInches;

    private final int kDrivePort;
    private final int kSteerPort;
    private final int kEncoderPort;

    public Mk4SwerveModuleConfig(double wheelRadiusInches, int kDrive, int kSteer, int kEncoder) {
        super(kDrive);
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
    public double getGearRatio() {
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
        return 5880.0 / 60.0 / getGearRatio() * 2 * Units.inchesToMeters(getWheelRadiusInches()) * Math.PI;
    }

    @Override
    public CANSparkMaxLowLevel.MotorType getMotorType() {
        return CANSparkMaxLowLevel.MotorType.kBrushless;
    }
}
