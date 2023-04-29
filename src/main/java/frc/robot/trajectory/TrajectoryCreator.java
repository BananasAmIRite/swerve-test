package frc.robot.trajectory;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.spline.Spline;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import frc.robot.lib.drive.swerve.commands.SwerveTrajectoryCommand;
import frc.robot.subsystems.drivetrain.Drivetrain;

import java.util.List;

import org.bananasamirite.robotmotionprofile.Waypoint;

public class TrajectoryCreator {
    private SwerveDriveKinematics kinematics;
    private SwerveDriveKinematicsConstraint kinematicsConstraint;
    private CentripetalAccelerationConstraint centripetalAccelerationConstraint;
    public TrajectoryCreator(SwerveDriveKinematics kinematics, SwerveDriveKinematicsConstraint kinematicsConstraint, CentripetalAccelerationConstraint centripetalAccelerationConstraint) {
        this.kinematics = kinematics;
        this.kinematicsConstraint = kinematicsConstraint;
        this.centripetalAccelerationConstraint = centripetalAccelerationConstraint;
    }

    public Trajectory create(List<Waypoint> waypoints, TrajectoryConfig config) {

        config.addConstraint(kinematicsConstraint).setKinematics(kinematics).addConstraint(centripetalAccelerationConstraint); 

        TrajectoryGenerator.ControlVectorList controlVectors = new TrajectoryGenerator.ControlVectorList();
        for (Waypoint w : waypoints) {
            controlVectors.add(new Spline.ControlVector(new double[] { // x
                w.getX(), w.getWeight()*Math.cos(w.getAngle()), 0
            }, new double[] { // y
                w.getY(), w.getWeight()*Math.sin(w.getAngle()), 0
            }));
        }
        return TrajectoryGenerator.generateTrajectory(controlVectors, config);
    }

    public Trajectory create(Waypoint[] waypoints, TrajectoryConfig config) {
        return create(List.of(waypoints), config); 
    }

    public SwerveTrajectoryCommand createCommand(Drivetrain drivetrain, List<Waypoint> waypoints, TrajectoryConfig config, boolean zero) {
                // Trajectory t = create(maxVel, maxAccel, waypoints, reversed);

                // TrajectoryConfig config = new TrajectoryConfig(maxVel, maxAccel).setKinematics(kinematics).addConstraint(voltageConstraint);
                // Trajectory t = TrajectoryGenerator.generateTrajectory(new Pose2d(), List.of(new Translation2d(1, 1)), new Pose2d(new Translation2d(2, 1), new Rotation2d()), config); 
                // return new RamseteCommand(t,  drivetrain::getPose, new RamseteController(), new SimpleMotorFeedforward(Constants.Trajectory.ksVolts, Constants.Trajectory.ksVoltSecondsPerMeter, Constants.Trajectory.kaVoltSecondsSquaredPerMeter), () -> drivetrain.getWheelSpeeds(), new PIDController(Constants.Trajectory.kPDriveVel, 0, 0), new PIDController(Constants.Trajectory.kPDriveVel, 0, 0), (a, b) -> drivetrain.tankDriveVolts(a, b), drivetrain); 
        
                // return new RamseteCommand(
                //     t,
                //     drivetrain::getPose,
                //     new RamseteController(),
                //     new SimpleMotorFeedforward(
                //         Constants.Trajectory.ksVolts,
                //         Constants.Trajectory.ksVoltSecondsPerMeter,
                //         Constants.Trajectory.kaVoltSecondsSquaredPerMeter),
                //         Constants.Trajectory.kDriveKinematics,
                //     drivetrain::getWheelSpeeds,
                //     new PIDController(Constants.Trajectory.kPDriveVel, 0, 0),
                //     new PIDController(Constants.Trajectory.kPDriveVel, 0, 0),
                //     // RamseteCommand passes volts to the callback
                //     drivetrain::tankDriveVolts,
                //     drivetrain);
                return new SwerveTrajectoryCommand(drivetrain, create(waypoints, config), zero); 
                // return new SimulateTrajectoryCommand(create(waypoints, config), drivetrain.getField());
    }

    public SwerveTrajectoryCommand createCommand(Drivetrain drivetrain, Waypoint[] waypoints, TrajectoryConfig config) {
        return createCommand(drivetrain, waypoints, config, true); 
    }

    public SwerveTrajectoryCommand createCommand(Drivetrain drivetrain, Waypoint[] waypoints, TrajectoryConfig config, boolean zero) {
        return createCommand(drivetrain, List.of(waypoints), config, zero); 
    }
}
