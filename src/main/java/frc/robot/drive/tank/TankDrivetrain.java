package frc.robot.drive.tank;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.drive.DriveSignal;
import frc.robot.drive.DrivetrainBase;

public class TankDrivetrain extends DrivetrainBase {

    private CANSparkMax[] leftMotors;
    private CANSparkMax[] rightMotors;

    private MotorControllerGroup leftMotorGroup;
    private MotorControllerGroup rightMotorGroup;

    private final TankDrivetrainConfig config;
    public TankDrivetrain(TankDrivetrainConfig config) {
        this.config = config;
    }

    private void setupMotors() {
        this.leftMotors = setupMotorSide(config.getLeftMotorConfigs());
        this.rightMotors = setupMotorSide(config.getRightMotorConfigs());

        this.leftMotorGroup = new MotorControllerGroup(this.leftMotors);
        this.rightMotorGroup = new MotorControllerGroup(this.rightMotors);
    }

    private CANSparkMax[] setupMotorSide(TankMotorConfig[] configs) {
        CANSparkMax[] motors = new CANSparkMax[configs.length];
        for (int i = 0; i < configs.length; i++) {
            motors[i] = new CANSparkMax(configs[i].getPort(), configs[i].getMotorType());
        }
        return motors;
    }

    @Override
    public void drive(DriveSignal signal) {
        if (signal.isFieldRelative() || signal.getSpeeds().vyMetersPerSecond != 0) throw new RuntimeException("Tank drive cannot handle field relative or strafing. ");
    }

    public void drive(ChassisSpeeds speeds) {
        // TODO: implement
    }

    @Override
    public void periodic() {

    }
}
