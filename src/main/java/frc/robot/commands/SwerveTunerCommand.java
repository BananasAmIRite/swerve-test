package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.SwerveModule;

public class SwerveTunerCommand extends CommandBase {
    SwerveModule module; 

    public SwerveTunerCommand(int turn, int drive, int encoder) {
        this.module = new SwerveModule(turn, drive, encoder); 
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("SwervekP", SmartDashboard.getNumber("SwervekP", Constants.DrivetrainConstants.kModuleTurn_P));
        SmartDashboard.putNumber("SwervekI", SmartDashboard.getNumber("SwervekI", Constants.DrivetrainConstants.kModuleTurn_I));
        SmartDashboard.putNumber("SwervekD", SmartDashboard.getNumber("SwervekD", Constants.DrivetrainConstants.kModuleTurn_D)); 
        SmartDashboard.putNumber("SwerveTurnSetpoint", SmartDashboard.getNumber("SwerveTurnSetpoint", 0)); 
    }

    @Override
    public void execute() {
        module.setTurnPID(
            SmartDashboard.getNumber("SwervekP", 0), 
            SmartDashboard.getNumber("SwervekI", 0), 
            SmartDashboard.getNumber("SwervekD", 0)
            );

        module.updateState(new SwerveModuleState(0, Rotation2d.fromDegrees(
            SmartDashboard.getNumber("SwerveTurnSetpoint", 0))
        ), SwerveModule.DriveState.OPEN_LOOP);

        SmartDashboard.putNumber("SwerveTurnMeasurement", module.getAngle().getDegrees()); 
        SmartDashboard.putNumber("SwerveError", module.turnPIDController.getPositionError()); 
        // SmartDashboard.putNumber("Swerve", module.turnPIDController.get); 
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}
