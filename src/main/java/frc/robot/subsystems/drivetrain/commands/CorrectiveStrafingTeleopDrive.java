package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.ControllerUtils;
import frc.robot.util.DriverController;

public class CorrectiveStrafingTeleopDrive extends CommandBase {
    DriverController controller;
    Drivetrain drivetrain;

    public CorrectiveStrafingTeleopDrive(DriverController controller, Drivetrain drivetrain){
        this.controller = controller;
        this.drivetrain = drivetrain;

        SmartDashboard.putNumber("corrective kP", SmartDashboard.getNumber("corrective kP", Constants.DrivetrainConstants.kCorrectiveStrafe_P));

        this.addRequirements(drivetrain);
    }

    @Override
    public void execute(){
        // get inputs, scale appropriately, calculate desired speed
        double inputOmega = -ControllerUtils.squareKeepSign(this.controller.getRightStickX()) * this.controller.getMaxRotation();
        double inputStrafeX =  
        -ControllerUtils.squareKeepSign(this.controller.getLeftStickX()) * this.controller.getMaxSpeed(); // horizontal
        double inputStrafeZ = -ControllerUtils.squareKeepSign(this.controller.getLeftStickY()) * this.controller.getMaxSpeed(); // vertical
        double inputStrafeSpeed = Math.sqrt(inputStrafeX * inputStrafeX + inputStrafeZ * inputStrafeZ);

        if(inputOmega == 0 || inputStrafeSpeed == 0){
            this.drivetrain.swerveDrive(new ChassisSpeeds(inputStrafeZ, inputStrafeX, inputOmega));
            return;
        }

        // calculate the angle of input strafe, rotate 90 degrees in opposite direction of turn
        double inputStrafeAngle = Math.atan2(inputStrafeZ, inputStrafeX);
        // TODO: plus or minus 90 degrees?
        double correctiveStrafeAngle = inputStrafeAngle + 0.5 * Math.PI * Math.signum(inputOmega);

        SmartDashboard.putNumber("input-strafe-speed", inputStrafeSpeed);
        SmartDashboard.putNumber("input-omega", inputOmega);
        SmartDashboard.putNumber("input-strafe-angle", inputStrafeAngle);
        SmartDashboard.putNumber("corrective-strafe-angle", correctiveStrafeAngle);

        double kP = SmartDashboard.getNumber("corrective kP", 0);

        // corrective strafe speed should be proportional to both the desired strafe speed as well as the desired angular speed
        double correctiveStrafeSpeed = kP * inputStrafeSpeed * Math.abs(inputOmega);
        double correctiveStrafeX = correctiveStrafeSpeed * Math.cos(correctiveStrafeAngle);
        double correctiveStrafeZ = correctiveStrafeSpeed * Math.sin(correctiveStrafeAngle);

        SmartDashboard.putNumber("corrective angle", Math.toDegrees(correctiveStrafeAngle)); 
        SmartDashboard.putNumber("input angle", Math.toDegrees(inputStrafeAngle)); 

        // add corrective strafe to input
        double netStrafeX = inputStrafeX + correctiveStrafeX;
        double netStrafeZ = inputStrafeZ + correctiveStrafeZ;

        ChassisSpeeds correctedSpeeds = new ChassisSpeeds(netStrafeZ, netStrafeX, inputOmega);

        this.drivetrain.swerveDrive(correctedSpeeds);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        this.drivetrain.swerveDrive(new ChassisSpeeds(0, 0, 0));
    }
}
