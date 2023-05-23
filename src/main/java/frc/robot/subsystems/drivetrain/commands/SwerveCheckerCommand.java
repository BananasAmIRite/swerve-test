package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.Constants;
import frc.robot.lib.drive.DriveSignal;

public class SwerveCheckerCommand extends CommandBase {
    Drivetrain drivetrain; 

    public SwerveCheckerCommand() {
        this.drivetrain = new Drivetrain(Constants.DrivetrainConstants.kDrivetrainConfig); 
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        this.drivetrain.drive(new DriveSignal(new ChassisSpeeds(1, 0, 0), false));
        // SmartDashboard.putNumber("Swerve", module.turnPIDController.get); 
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}
