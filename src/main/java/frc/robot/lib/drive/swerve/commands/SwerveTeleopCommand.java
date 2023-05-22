package frc.robot.lib.drive.swerve.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.lib.drive.DriveCommand;
import frc.robot.lib.drive.DriveSignal;
import frc.robot.lib.drive.swerve.SwerveDrivetrain;
import frc.robot.lib.util.Controller;
import frc.robot.lib.util.ControllerUtils;

import java.util.function.Supplier;

public class SwerveTeleopCommand extends DriveCommand {
    private final SwerveDrivetrain drivetrain;
    private final Controller controller;

    private final double maxSpeed;
    private final double maxRotation;

    private final SlewRateLimiter forwardRateLimiter;
    private final SlewRateLimiter strafeRateLimiter;

    private final SlewRateLimiter turnRateLimiter;

    private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;
    public SwerveTeleopCommand(SwerveDrivetrain drivetrain, Controller controller, double maxSpeed, double maxRotation, double forwardSlewRate, double strafeSlewRate, double turnSlewRate, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
        super(drivetrain);
        this.drivetrain = drivetrain;
        this.controller = controller;

        this.maxSpeed = maxSpeed;
        this.maxRotation = maxRotation;

        this.forwardRateLimiter = new SlewRateLimiter(forwardSlewRate);
        this.strafeRateLimiter = new SlewRateLimiter(strafeSlewRate);
        this.turnRateLimiter = new SlewRateLimiter(turnSlewRate);

        this.chassisSpeedsSupplier = chassisSpeedsSupplier;

        this.addRequirements(drivetrain);
    }

    @Override
    public void initialize() {}

    @Override
    public DriveSignal update() {
        // deadzone-only values
        double throttleForward = -controller.getLeftStickY();
        double throttleStrafe = -controller.getLeftStickX();
        double throttleTurn = -controller.getRightStickX();

        double speedForward = ControllerUtils.squareKeepSign(throttleForward) * maxSpeed;
        double speedStrafe = ControllerUtils.squareKeepSign(throttleStrafe) * maxSpeed;
        double speedTurn = ControllerUtils.squareKeepSign(throttleTurn) * maxRotation;

        ChassisSpeeds speeds = new ChassisSpeeds(
            speedForward, 
            speedStrafe,
            speedTurn
                // forwardRateLimiter.calculate(speedForward),
                // strafeRateLimiter.calculate(speedStrafe),
                // turnRateLimiter.calculate(speedTurn)
        );

        // ChassisSpeeds oldSpeeds = chassisSpeedsSupplier.get();
        // if (oldSpeeds != null) {
        //     forwardRateLimiter.reset(oldSpeeds.vxMetersPerSecond);
        //     strafeRateLimiter.reset(oldSpeeds.vyMetersPerSecond);
        //     turnRateLimiter.reset(oldSpeeds.omegaRadiansPerSecond);
        // }

        return new DriveSignal(speeds, true);
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
