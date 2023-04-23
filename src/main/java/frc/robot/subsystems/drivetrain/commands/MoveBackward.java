package frc.robot.subsystems.drivetrain.commands;

import java.util.List;

import org.bananasamirite.robotmotionprofile.Waypoint;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.Constants;
import frc.robot.commands.trajectory.SwerveTrajectoryCommand;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class MoveBackward extends SwerveTrajectoryCommand {

    public MoveBackward(Drivetrain drivetrain, double distanceMeters, double maxVel, double maxAccel) {
        super(drivetrain, Constants.Trajectory.trajectoryCreator.create(List.of(new Waypoint(0, 0, 0, 1, 1), new Waypoint(-distanceMeters, 0, 0, 1, 1)), new TrajectoryConfig(maxVel, maxAccel).setReversed(true)));
    }
    
    public MoveBackward(Drivetrain drivetrain, double distanceMeters) {
        this(drivetrain, distanceMeters, Constants.Trajectory.kMaxVelocityMetersPerSecond, Constants.Trajectory.kMaxAccelerationMetersPerSecondSquared); 
    }
}
