// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.leds.addressable.patterns.LEDPattern;
import frc.robot.subsystems.leds.addressable.patterns.SolidLEDPattern;
import frc.robot.trajectory.TrajectoryCreator;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double kLoopDuration = 0.02; 
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DrivetrainConstants {
    public static class FrontLeft {
      public static final int kRotate = 0; 
      public static final int kDrive = 1; 
    }
    public static class FrontRight {
      public static final int kRotate = 2; 
      public static final int kDrive = 3; 
    }
    public static class BackLeft {
      public static final int kRotate = 4; 
      public static final int kDrive = 5; 
    }
    public static class BackRight {
      public static final int kRotate = 6; 
      public static final int kDrive = 7; 
    }

    public static final double kGearRatio = 1; 
    public static final double kWheelRadiusInches = 3; 
    public static final double kMetersPerRot = Units.inchesToMeters(2 * Math.PI * kWheelRadiusInches / kGearRatio);
    public static final double kMetersPerSecondPerRPM = kMetersPerRot / 60;

    public static final double kRotateGearRatio = 1; 
    public static final double kDegreesPerRot = 360/kGearRatio;
    public static final double kDegreesPerSecondPerRPM = kDegreesPerRot/60; 

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics( // TODO: may need to switch up
      new Translation2d(0.5, 0.5), // front left
      new Translation2d(0.5, -0.5), // front right
      new Translation2d(-0.5, 0.5), // back left
      new Translation2d(-0.5, -0.5) // back right
    ); 

    public static final double kMaxSpeedMetersPerSecond = 3.21; // max velocity in m/s
    public static final double kMaxAccelerationMetersPerSecondSquared = 2.54; 
    public static final double kMaxRotationRadPerSecond = 2 * Math.PI; // max angular velocity in rad/s
    public static final double kMaxRotationAccelerationRadPerSecondSquared = Math.PI; // max angular acceleration in rad/s^2

    // can probably be found by just running the whole drivetrain sysid as a differential drive system (lock swerve modules)
    public static final double ksVolts = 0; 
    public static final double kvVoltSecondsPerMeter = 0; 
    public static final double kaVoltSecondsSquaredPerMeter = 0; 

    // // per-swerve-module turn feedforward values; calculate by running SysID on one of the turn swerve modules
    // // make sure to run on the carpet
    // public static final double ksTurnVolts = 0; 
    // public static final double kvTurnVoltSecondsPerMeter = 0; 
    // public static final double kaTurnVoltSecondsSquaredPerMeter = 0; 


    // drivetrain sysid (lock wheels)
    public static final double kModuleDrive_P = 0; 
    public static final double kModuleDrive_I = 0; 
    public static final double kModuleDrive_D = 0; 

    // found from sysid for one of the turn modules or tune by yourself
    public static final double kModuleTurn_P = 0; 
    public static final double kModuleTurn_I = 0; 
    public static final double kModuleTurn_D = 0; 

    // turn in place pid
    public static final double kTurn_P = 0; 
    public static final double kTurn_I = 0; 
    public static final double kTurn_D = 0; 
    public static final double kTurn_FF = 0; 
    public static final double kTurnErrorThreshold = 0; 
    public static final double kTurnVelocityThreshold = 0; 

    // just set the motor to 1.0 speed and see how fast the wheel spins from encoders
    public static final double kMaxAttainableModuleSpeedMetersPerSecond = 0; 

    // TODO: tune these
    public static final int kDriveCurrentLimit = 40; 
    public static final int kTurnCurrentLimit = 40; 

    public static final double kForwardSlewRate = kMaxAccelerationMetersPerSecondSquared; 
    public static final double kStrafeSlewRate = kMaxAccelerationMetersPerSecondSquared; 
    public static final double kTurnSlewRate = kMaxRotationAccelerationRadPerSecondSquared; 
  }

  public static class Trajectory {
    public static final double kDrive_P = DrivetrainConstants.kModuleDrive_P; 
    public static final double kDrive_I = DrivetrainConstants.kModuleDrive_I; 
    public static final double kDrive_D = DrivetrainConstants.kModuleDrive_D;

    // drivetrain angular sysid
    public static final double kOmega_P = 0; 
    public static final double kOmega_I = 0; 
    public static final double kOmega_D = 0; 

    public static final double kMaxCentripetalAcceleration = 0.8; 

    public static final TrajectoryCreator trajectoryCreator = new TrajectoryCreator(DrivetrainConstants.kDriveKinematics, new SwerveDriveKinematicsConstraint(DrivetrainConstants.kDriveKinematics, DrivetrainConstants.kMaxAttainableModuleSpeedMetersPerSecond), new CentripetalAccelerationConstraint(kMaxCentripetalAcceleration)); 
  }

  public static class LEDs {
    public static final Color kDefaultColor = Color.kOrange;

    public static final int kLedPort = 0; 
    public static final int kLedLength = 40; 

    public static final int kLed1Start = 0; 
    public static final int kLed1End = 20; 
    public static final int kLed2Start = 20; 
    public static final int kLed2End = 40; 

    public static final class Patterns {
      public static final LEDPattern kDefault = new SolidLEDPattern(Color.kOrange); 
    }
}
}
