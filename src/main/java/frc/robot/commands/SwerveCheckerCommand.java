package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.SwerveModule;

public class SwerveCheckerCommand extends CommandBase {
    Drivetrain drivetrain; 

    public SwerveCheckerCommand() {
        this.drivetrain = new Drivetrain(); 
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        this.drivetrain.swerveDrive(1, 0, 0);
        // SmartDashboard.putNumber("Swerve", module.turnPIDController.get); 
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}
