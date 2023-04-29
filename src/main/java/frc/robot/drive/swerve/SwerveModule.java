package frc.robot.drive.swerve;

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
    
    private CANSparkMax steerMotor; 
    private CANSparkMax driveMotor;
    
    private SparkMaxPIDController turnPIDController; 
    private SparkMaxPIDController drivePIDController; 

    // represents the true, uninverted heading of the drive motor
    private CANCoder absoluteSteerEncoder;
    
    private RelativeEncoder steerEncoder; 

    private RelativeEncoder driveEncoder; 

    private SwerveModuleConfig config; 


    private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(
        Constants.DrivetrainConstants.ksVolts, 
        Constants.DrivetrainConstants.kvVoltSecondsPerMeter, 
        Constants.DrivetrainConstants.kaVoltSecondsSquaredPerMeter
    ); 

    public SwerveModule(SwerveModuleConfig config) {
        this.steerMotor = new CANSparkMax(config.getSteerPort(), MotorType.kBrushless); 
        this.driveMotor = new CANSparkMax(config.getDrivePort(), MotorType.kBrushless); 

        this.absoluteSteerEncoder = new CANCoder(config.getEncoderPort()); // this.turnMotor.getAbsoluteEncoder(Type.kDutyCycle); 
        this.steerEncoder = this.steerMotor.getEncoder(); 
        this.driveEncoder = this.driveMotor.getEncoder(); 

        this.turnPIDController = this.steerMotor.getPIDController(); 
        this.drivePIDController = this.driveMotor.getPIDController(); 

        this.config = config; 

        configureMotors();
        configureEncoders();
        configurePIDControllers();
        

        // in case of brownout
        this.steerMotor.burnFlash(); 
        this.driveMotor.burnFlash(); 
    }

    private void configureMotors() {
        this.driveMotor.setSmartCurrentLimit(config.getDriveCurrentLimit()); 
        this.driveMotor.setIdleMode(IdleMode.kBrake); 
        this.driveMotor.enableVoltageCompensation(12); 

        this.steerMotor.setSmartCurrentLimit(config.getSteerCurrentLimit()); 
        this.steerMotor.setIdleMode(IdleMode.kBrake); 
        this.steerMotor.enableVoltageCompensation(12); 
    }

    private void configureEncoders() {
        this.steerEncoder.setPositionConversionFactor(config.getSteerPositionConversion()); 
        this.steerEncoder.setVelocityConversionFactor(config.getSteerVelocityConversion());

        this.steerEncoder.setPosition(this.absoluteSteerEncoder.getAbsolutePosition()); 
        
        this.driveEncoder.setPositionConversionFactor(config.getDrivePositionConversion());
        this.driveEncoder.setVelocityConversionFactor(config.getDriveVelocityConversion()); 
    }

    private void configurePIDControllers() {
        this.turnPIDController.setPositionPIDWrappingEnabled(true); 
        this.turnPIDController.setPositionPIDWrappingMinInput(0); 
        this.turnPIDController.setPositionPIDWrappingMinInput(360); 

        this.turnPIDController.setP(Constants.DrivetrainConstants.kModuleTurn_P);
        this.turnPIDController.setI(Constants.DrivetrainConstants.kModuleTurn_I); 
        this.turnPIDController.setD(Constants.DrivetrainConstants.kModuleTurn_D); 
        
        this.drivePIDController.setP(Constants.DrivetrainConstants.kModuleDrive_P);
        this.drivePIDController.setI(Constants.DrivetrainConstants.kModuleDrive_I); 
        this.drivePIDController.setD(Constants.DrivetrainConstants.kModuleDrive_D); 
    }

    public void updateState(SwerveModuleState state, DriveState driveType) {
        this.state = state; 
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getAngle()); 

        if (driveType == DriveState.OPEN_LOOP) updateOpenLoopDriveState(optimizedState.speedMetersPerSecond); 
         else updateClosedLoopDriveState(optimizedState.speedMetersPerSecond);

         updateTurnState(state.angle);
    }

    private void updateOpenLoopDriveState(double speed) {
        double percent = MathUtil.clamp(speed / config.getMaxAttainableSpeed(), -1, 1); 
        driveMotor.set(percent);
    }
    
    private void updateClosedLoopDriveState(double speed) {
        drivePIDController.setReference(speed, ControlType.kVelocity, 0, driveFeedforward.calculate(speed)); 
    }

    private void updateTurnState(Rotation2d turn) {
        turnPIDController.setReference(turn.getDegrees(), ControlType.kPosition); 
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(this.driveEncoder.getPosition()); 
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
