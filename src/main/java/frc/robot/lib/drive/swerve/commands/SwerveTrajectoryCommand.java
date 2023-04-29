package frc.robot.lib.drive.swerve.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.lib.drive.DriveCommand;
import frc.robot.lib.drive.DriveSignal;
import frc.robot.lib.drive.swerve.SwerveDrivetrain;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class SwerveTrajectoryCommand extends DriveCommand
{
    private final SwerveDrivetrain subsystem;
    private final HolonomicDriveController controller;

    private final Timer timer;

    private final Trajectory trajectory;
    private final double maxTime;

    private double startTime;
    private double prevTime;

    private SwerveModuleState[] prevSpeeds; 

    private final boolean zero;

    public SwerveTrajectoryCommand(SwerveDrivetrain drivetrain, Trajectory trajectory) {
        this(drivetrain, trajectory, true);
    }

    public SwerveTrajectoryCommand(SwerveDrivetrain drivetrain, Trajectory trajectory, boolean zero)
    {
        super(drivetrain);
        this.subsystem = drivetrain;
        this.timer = new Timer();
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);

        this.zero = zero; 

        this.trajectory = trajectory;
        this.maxTime = trajectory.getTotalTimeSeconds();

        // TODO: figure out where to put these
        this.controller = new HolonomicDriveController(
            new PIDController(
            Constants.Trajectory.kDrive_P, Constants.Trajectory.kDrive_I, Constants.Trajectory.kDrive_D
            ), 
            new PIDController(
                Constants.Trajectory.kDrive_P, Constants.Trajectory.kDrive_I, Constants.Trajectory.kDrive_D
            ), 
            new ProfiledPIDController(
                Constants.Trajectory.kOmega_P, Constants.Trajectory.kOmega_I, Constants.Trajectory.kOmega_D, 
                new Constraints(Math.PI * 2, Math.PI)
            )
        ); 
    }

    @Override
    public void initialize() {
        prevTime = -1;
        timer.reset();
        timer.start();

        Trajectory.State initialState = this.trajectory.sample(0);
        
        prevSpeeds = Constants.DrivetrainConstants.kDriveKinematics.toSwerveModuleStates(
                new ChassisSpeeds(
                        initialState.velocityMetersPerSecond,
                        0,
                        initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond
                )
        );
        
        // figure out a way that doesn't involve resetting odometry.
        // if we're using field-relative odometry though, we should be fine just not resetting as long as the setpoints are accurate to the field-relative coordinate system
        if (zero) subsystem.resetOdometry(initialState.poseMeters); 

    }

    @Override
    public DriveSignal update() {
        double curTime = timer.get();

        Trajectory.State state = this.trajectory.sample(curTime);

        if (prevTime < 0) {
            prevTime = curTime;
            return new DriveSignal(new ChassisSpeeds(), false);
        }

        ChassisSpeeds speeds = controller.calculate(subsystem.getPose(), state, Rotation2d.fromDegrees(0));

        prevSpeeds = subsystem.getKinematics().toSwerveModuleStates(speeds);
        prevTime = curTime;

        return new DriveSignal(speeds, false);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished()
    {
        return timer.hasElapsed(maxTime);
    }
}
