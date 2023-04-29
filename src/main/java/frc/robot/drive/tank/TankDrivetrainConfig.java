package frc.robot.drive.tank;

import frc.robot.drive.DrivetrainConfig;

import java.util.Collections;
import java.util.List;

public class TankDrivetrainConfig extends DrivetrainConfig {

    private final double trackWidth;
    private final double wheelBase;

    private TankMotorConfig[] leftMotorConfigs;
    private TankMotorConfig[] rightMotorConfigs;

    private double maxSpeed;

    public TankDrivetrainConfig(double trackWidth, double driveBase) {
        this.trackWidth = trackWidth;
        this.wheelBase = driveBase;
    }

    public void setLeftMotorConfigs(TankMotorConfig... configs) {
        this.leftMotorConfigs = configs;
        this.maxSpeed = Collections.min(List.of(configs), (o1, o2) -> (int) (o1.getMaxAttainableSpeed() - o2.getMaxAttainableSpeed())).getMaxAttainableSpeed();
    }

    public void setRightMotorConfigs(TankMotorConfig... configs) {
        this.rightMotorConfigs = configs;
        this.maxSpeed = Collections.min(List.of(configs), (o1, o2) -> (int) (o1.getMaxAttainableSpeed() - o2.getMaxAttainableSpeed())).getMaxAttainableSpeed();
    }

    public TankMotorConfig[] getLeftMotorConfigs() {
        return leftMotorConfigs;
    }

    public TankMotorConfig[] getRightMotorConfigs() {
        return rightMotorConfigs;
    }

    @Override
    public double getTrackWidthMeters() {
        return trackWidth;
    }

    @Override
    public double getWheelBaseMeters() {
        return wheelBase;
    }

    @Override
    public double getMaxDriveSpeed() {
        return this.maxSpeed;
    }
}
