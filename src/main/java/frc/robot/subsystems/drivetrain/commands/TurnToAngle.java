package frc.robot.subsystems.drivetrain.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.lib.drive.DriveSignal;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class TurnToAngle extends CommandBase {
        private Drivetrain drivetrain;  
        private PIDController pidController; 
        private DoubleSupplier setpoint; 

        public TurnToAngle(Drivetrain drivetrain, double angle) {
            this(drivetrain, () -> angle); 
        }
    
        public TurnToAngle(Drivetrain drivetrain, DoubleSupplier setpoint) {
            this.pidController = new PIDController(
                Constants.DrivetrainConstants.kTurn_P,
                Constants.DrivetrainConstants.kTurn_I,
                Constants.DrivetrainConstants.kTurn_D
            ); 
            this.pidController.setTolerance(Constants.DrivetrainConstants.kTurnErrorThreshold);
            this.pidController.enableContinuousInput(-180.0, 180.0);
            
            SmartDashboard.putNumber("Angular kP", SmartDashboard.getNumber("Angular kP", Constants.DrivetrainConstants.kTurn_P)); 
            SmartDashboard.putNumber("Angular kI", SmartDashboard.getNumber("Angular kI", Constants.DrivetrainConstants.kTurn_I)); 
            SmartDashboard.putNumber("Angular kD", SmartDashboard.getNumber("Angular kD", Constants.DrivetrainConstants.kTurn_D)); 
            SmartDashboard.putNumber("Angular kFF", SmartDashboard.getNumber("Angular kFF", Constants.DrivetrainConstants.kTurn_FF)); 

            SmartDashboard.putBoolean("Angular Running", true);
    
            this.drivetrain = drivetrain;
            this.setpoint = setpoint; 

            addRequirements(drivetrain);
            System.out.println("hi");
        }
    
        @Override
        public void execute() {
            // For tuning
            // System.out.println(SmartDashboard.getNumber("Angular kP", 0.0));
            // this.pidController.setPID(
            //     SmartDashboard.getNumber("Angular kP", 0.0),
            //     SmartDashboard.getNumber("Angular kI", 0.0),
            //     SmartDashboard.getNumber("Angular kD", 0.0)
            // );

            // this.pidController.setPID(
            //     SmartDashboard.getNumber("Angular kP", 0.0),
            //     SmartDashboard.getNumber("Angular kI", 0.0),
            //     SmartDashboard.getNumber("Angular kD", 0.0)
            // );

            double output = pidController.calculate(drivetrain.getYaw(), setpoint.getAsDouble()); 
            //  + SmartDashboard.getNumber("Angular kFF", 0) * Math.signum(pidController.getPositionError());
            if (Math.abs(output) < Constants.DrivetrainConstants.kTurn_FF) output = Math.signum(output) * Constants.DrivetrainConstants.kTurn_FF;  
            output = MathUtil.clamp(output, -1, 1); 
            SmartDashboard.putNumber("setpoint", setpoint.getAsDouble()); 
            SmartDashboard.putNumber("error", pidController.getPositionError()); 
            // SmartDashboard.putNumber("", output); 
            SmartDashboard.putNumber("output", output); 
            SmartDashboard.putNumber("angle velo", pidController.getVelocityError());
            drivetrain.drive(new DriveSignal(new ChassisSpeeds(0, 0, output), false));
        }
    
        @Override
        public void end(boolean interrupted) {
            SmartDashboard.putBoolean("Angular Running", false);
            drivetrain.brake();
        }
    
        @Override
        public boolean isFinished() {
            return this.pidController.atSetpoint();
        }
}