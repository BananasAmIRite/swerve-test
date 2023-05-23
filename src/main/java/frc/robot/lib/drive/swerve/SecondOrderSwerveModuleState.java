package frc.robot.lib.drive.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SecondOrderSwerveModuleState extends SwerveModuleState {
    public double angularVelocityRadiansPerSecond;

    public SecondOrderSwerveModuleState(
            double speedMetersPerSecond,
            Rotation2d angle,
            double angularVelocityRadiansPerSecond) {
        super(speedMetersPerSecond, angle);
        this.angularVelocityRadiansPerSecond = angularVelocityRadiansPerSecond;
    }

    public SecondOrderSwerveModuleState() {
        this(0, new Rotation2d(), 0);
    }
}