package frc.robot.lib.drive;

import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.util.Units;

public abstract class WheelConfig {
    private final int port;

    public WheelConfig(int port) {
        this.port = port;
    }
    public abstract double getWheelRadiusInches();

    public abstract double getGearRatio();

    public abstract CANSparkMaxLowLevel.MotorType getMotorType();

    public double getPositionConversion() {
        return Units.inchesToMeters(2 * Math.PI * getWheelRadiusInches() / getGearRatio());
    }

    /**
     * Drive velocity conversion in Meters per Second per RPM
     */
    public double getVelocityConversion() {
        return getPositionConversion() / 60;
    }

    public int getPort() {
        return this.port;
    }

    public abstract double getMaxAttainableSpeed();
}
