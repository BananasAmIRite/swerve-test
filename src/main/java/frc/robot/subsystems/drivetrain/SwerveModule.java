package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveModule {

    private SwerveModuleState state;
    
    private CANSparkMax turnMotor; 
    private CANSparkMax driveMotor;
    
    private SwerveTurnPIDController turnPIDController; 
    private SparkMaxPIDController drivePIDController; 

    // represents the true, uninverted heading of the drive motor
    private CANCoder absoluteTurnEncoder;

    private RelativeEncoder driveEncoder; 


    private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(
        Constants.DrivetrainConstants.ksVolts, 
        Constants.DrivetrainConstants.kvVoltSecondsPerMeter, 
        Constants.DrivetrainConstants.kaVoltSecondsSquaredPerMeter
    ); 

    public SwerveModule(int kTurn, int kDrive, int kTurnEncoder) {
        this.turnMotor = new CANSparkMax(kTurn, MotorType.kBrushless); 
        this.driveMotor = new CANSparkMax(kDrive, MotorType.kBrushless); 

        this.absoluteTurnEncoder = new CANCoder(kTurnEncoder); // this.turnMotor.getAbsoluteEncoder(Type.kDutyCycle); 
        this.driveEncoder = this.driveMotor.getEncoder(); 

        this.turnPIDController = new SwerveTurnPIDController(kTurnEncoder, 0, 0, 0); 
        this.drivePIDController = this.driveMotor.getPIDController(); 

        configureMotors();
        configureEncoders();
        configurePIDControllers();

        // in case of brownout
        this.turnMotor.burnFlash(); 
        this.driveMotor.burnFlash(); 
    }

    private void configureMotors() {
        this.driveMotor.setSmartCurrentLimit(Constants.DrivetrainConstants.kDriveCurrentLimit); 
        this.driveMotor.setIdleMode(IdleMode.kBrake); 
        this.driveMotor.enableVoltageCompensation(12); 

        this.turnMotor.setSmartCurrentLimit(Constants.DrivetrainConstants.kTurnCurrentLimit); 
        this.turnMotor.setIdleMode(IdleMode.kBrake); 
        this.turnMotor.enableVoltageCompensation(12); 
    }

    private void configureEncoders() {

        System.out.println(Constants.DrivetrainConstants.kDegreesPerRot);
        
        this.driveEncoder.setPositionConversionFactor(Constants.DrivetrainConstants.kMetersPerRot);
        this.driveEncoder.setVelocityConversionFactor(Constants.DrivetrainConstants.kMetersPerSecondPerRPM); 
    }

    private void configurePIDControllers() {
        setTurnPID(
            Constants.DrivetrainConstants.kModuleTurn_P, 
            Constants.DrivetrainConstants.kModuleTurn_I, 
            Constants.DrivetrainConstants.kModuleTurn_D
            );
        
        this.drivePIDController.setP(Constants.DrivetrainConstants.kModuleDrive_P);
        this.drivePIDController.setI(Constants.DrivetrainConstants.kModuleDrive_I); 
        this.drivePIDController.setD(Constants.DrivetrainConstants.kModuleDrive_D); 
    }

    public void setTurnPID(double p, double i, double d) {
        this.turnPIDController.setP(p);
        this.turnPIDController.setI(i);
        this.turnPIDController.setD(d);
    }

    public void updateState(SwerveModuleState state, DriveState driveType) {
        this.state = state; 
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getAngle()); 

        if (driveType == DriveState.OPEN_LOOP) updateOpenLoopDriveState(optimizedState.speedMetersPerSecond); 
         else updateClosedLoopDriveState(optimizedState.speedMetersPerSecond);

         updateTurnState(state.angle);
    }

    private void updateOpenLoopDriveState(double speed) {
        double percent = MathUtil.clamp(speed / Constants.DrivetrainConstants.kMaxAttainableSpeedMetersPerSecond, -1, 1); 
        driveMotor.set(percent);
    }
    
    private void updateClosedLoopDriveState(double speed) {
        drivePIDController.setReference(speed, ControlType.kVelocity, 0, driveFeedforward.calculate(speed)); 
    }

    private void updateTurnState(Rotation2d turn) {
        turnPIDController.setSetpoint(turn.getDegrees()); 
        turnMotor.set(turnPIDController.calculate());
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(this.absoluteTurnEncoder.getAbsolutePosition()); 
    }

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(this.driveEncoder.getVelocity(), Rotation2d.fromDegrees(this.driveEncoder.getPosition())); 
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(this.driveEncoder.getPosition(), Rotation2d.fromDegrees(this.driveEncoder.getPosition())); 
    }

    public SwerveModuleState getReferenceState() {
        return this.state; 
    }

    public static enum DriveState {
        OPEN_LOOP, 
        CLOSED_LOOP
    }
}
