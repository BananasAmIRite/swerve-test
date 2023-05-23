// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveTunerCommand;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.leds.addressable.LED;
import frc.robot.subsystems.leds.addressable.patterns.LEDPattern;
import frc.robot.util.Controller;
import frc.robot.util.DriverController;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final DriverController driverController =
      new DriverController(0);

  // private Command command = new SwerveTunerCommand(Constants.DrivetrainConstants.BackLeft.kRotate, Constants.DrivetrainConstants.BackLeft.kDrive, Constants.DrivetrainConstants.BackLeft.kRotEncoder); 

  private SendableChooser<LEDPattern> patterns = new SendableChooser<>(); 

  private final LED leds = new LED(); 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    /***** apriltag transformation stuff
     * the idea is that, when apriltags are out of the camera, use the odometry feedback from the 
     * drivetrain as data to estimate where the apriltag is
     */
    Pose2d oldAprTagPose = new Pose2d(3, 0, Rotation2d.fromDegrees(180)); 
    Pose2d oldOdo = new Pose2d(0, 0, new Rotation2d()); 
    Pose2d newOdo = new Pose2d(0, -4, Rotation2d.fromDegrees(57)); 

    Pose2d oldFieldRelative = oldOdo.transformBy(new Transform2d(new Pose2d(), oldAprTagPose)); 
    Pose2d rel2 = oldFieldRelative.relativeTo(newOdo); 

    Pose2d res = rel2; 
    System.out.println(res);
    // driverController.setChassisSpeedsSupplier(drivetrain::getChassisSpeeds); // comment in simulation
    drivetrain.setDefaultCommand(new RunCommand(() -> {
      ChassisSpeeds speeds = driverController.getDesiredChassisSpeeds(); 
      SmartDashboard.putNumber("x", speeds.vxMetersPerSecond); 
      SmartDashboard.putNumber("y", speeds.vyMetersPerSecond); 
      SmartDashboard.putNumber("rotation", speeds.omegaRadiansPerSecond); 
      drivetrain.swerveDrive(speeds);
    }, drivetrain));

    patterns.addOption("Idle", Constants.LEDs.Patterns.kIdle);
    patterns.addOption("Rainbow", Constants.LEDs.Patterns.kBalanceFinished);
    patterns.addOption("Dead", Constants.LEDs.Patterns.kDead);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    Controller.onPress(driverController.Y, new InstantCommand(() -> {
      drivetrain.zeroYaw();
    }));
  }

  public void doSendables() {
    SmartDashboard.putData(patterns);
    leds.setPattern(patterns.getSelected());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null; 
  }
}
