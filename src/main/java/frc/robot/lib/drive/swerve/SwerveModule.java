package frc.robot.lib.drive.swerve;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.lib.util.Encoder;

public class SwerveModule {

    private SwerveModuleState state;
    
    private final CANSparkMax steerMotor;
    private final CANSparkMax driveMotor;
    
    private final SwerveTurnPIDController steerPIDController; 
    private final SparkMaxPIDController drivePIDController; 

    // represents the true, uninverted heading of the drive motor
    private final CANCoder absoluteSteerEncoder;
    
    private final Encoder steerEncoder;

    private final Encoder driveEncoder;

    private final SwerveModuleConfig config;


    private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(
        Constants.DrivetrainConstants.ksVolts, 
        Constants.DrivetrainConstants.kvVoltSecondsPerMeter, 
        Constants.DrivetrainConstants.kaVoltSecondsSquaredPerMeter
    ); 

    public SwerveModule(SwerveModuleConfig config) {
        this.steerMotor = new CANSparkMax(config.getSteerPort(), config.getMotorType());
        this.driveMotor = new CANSparkMax(config.getDrivePort(), config.getMotorType());

        this.absoluteSteerEncoder = new CANCoder(config.getEncoderPort()); // this.turnMotor.getAbsoluteEncoder(Type.kDutyCycle); 
        this.steerEncoder = new Encoder(this.steerMotor.getEncoder());
        this.driveEncoder = new Encoder(this.driveMotor.getEncoder());

        this.steerPIDController = new SwerveTurnPIDController(this.absoluteSteerEncoder, 0, 0, 0); 
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
        this.steerEncoder.getEncoder().setPositionConversionFactor(config.getSteerPositionConversion());
        this.steerEncoder.getEncoder().setVelocityConversionFactor(config.getSteerVelocityConversion());

        this.steerEncoder.setPosition(this.absoluteSteerEncoder.getAbsolutePosition()); 
        
        this.driveEncoder.getEncoder().setPositionConversionFactor(config.getPositionConversion());
        this.driveEncoder.getEncoder().setVelocityConversionFactor(config.getVelocityConversion());
    }

    private void configurePIDControllers() {
        // TODO: dont do this
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
        this.steerPIDController.setPID(p, i, d);
    }

    public void updateState(SwerveModuleState state, DriveState driveType) {
        this.state = state; 
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getAngle()); 

        if (driveType == DriveState.OPEN_LOOP) updateOpenLoopDriveState(optimizedState.speedMetersPerSecond); 
         else updateClosedLoopDriveState(optimizedState.speedMetersPerSecond);

         updateTurnState(optimizedState.angle);
    }

    private void updateOpenLoopDriveState(double speed) {
        double percent = MathUtil.clamp(speed / config.getMaxAttainableSpeed(), -1, 1); 
        driveMotor.set(percent);
    }
    
    private void updateClosedLoopDriveState(double speed) {
        drivePIDController.setReference(speed, ControlType.kVelocity, 0, driveFeedforward.calculate(speed)); 
    }

    private void updateTurnState(Rotation2d turn) {
        steerPIDController.setSetpoint(turn.getDegrees()); 
        steerMotor.set(steerPIDController.calculate());
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(this.driveEncoder.getPosition()); 
    }

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(this.driveEncoder.getEncoder().getVelocity(), Rotation2d.fromDegrees(this.driveEncoder.getPosition()));
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
