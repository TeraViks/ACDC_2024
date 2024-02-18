// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static final double kDt = 0.02;
  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kMaxRadPerSec = DriveConstants.kMaxAngularSpeedRadiansPerSecond;
    public static final double kMaxMetersPerSec = DriveConstants.kMaxSpeedMetersPerSecond;
    
    public static final int kA = 1;
    public static final int kB = 2;
    public static final int kX = 3;
    public static final int kY = 4;
    public static final int kLeftBumper = 5;
    public static final int kRightBumper = 6;
    public static final int kBack = 7;
    public static final int kStart = 8;
    public static final int kLeftJoy = 9;
    public static final int kRightJoy = 10;

    public static int kLeftJoyXAxis = 0;
    public static int kLeftJoyYAxis = 1;
    public static int kLeftTriggerAxis = 2;
    public static int kRightTriggerAxis = 3;
    public static int kRightJoyXAxis = 4;
    public static int kRightJoyYAxis = 5;

    public static final int kJoystickTargetNoteButton = kY;

    public static final double kDebounceSeconds = 0.01;

    public static final double kJoystickDeadband = 0.05;
  }

  public static class IntakeConstants {
    //TODO: Assign a real CAN ID
    public static final int kIntakeMotorID = 0;
    public static final int kIntakeCurrentLimit = 40;
  }

  public static class DriveConstants {
    public static final int kFrontLeftDriveMotorPort = 1; //Q1
    public static final int kRearLeftDriveMotorPort = 2; //Q2
    public static final int kRearRightDriveMotorPort = 3; //Q3
    public static final int kFrontRightDriveMotorPort = 4; //Q4

    public static final int kFrontLeftTurningMotorPort = 5; //Q1
    public static final int kRearLeftTurningMotorPort = 6; //Q2
    public static final int kRearRightTurningMotorPort = 7; //Q3
    public static final int kFrontRightTurningMotorPort = 8; //Q4

    public static final int kFrontLeftTurningEncoderPort = 9; //Q1
    public static final int kRearLeftTurningEncoderPorts = 10; //Q2
    public static final int kRearRightTurningEncoderPorts = 11; //Q3
    public static final int kFrontRightTurningEncoderPorts = 12; //Q4

    public static final boolean kFrontLeftTurningMotorReversed = true; //Q1
    public static final boolean kRearLeftTurningMotorReversed = true; //Q2
    public static final boolean kRearRightTurningMotorReversed = true; //Q3
    public static final boolean kFrontRightTurningMotorReversed = true; //Q4

    public static final boolean kFrontLeftDriveReversed = true; //Q1
    public static final boolean kRearLeftDriveReversed = true; //Q2
    public static final boolean kRearRightDriveReversed = true; //Q3
    public static final boolean kFrontRightDriveReversed = true; //Q4

    public static final boolean kFrontLeftEncoderReversed = false; //Q1
    public static final boolean kRearLeftEncoderReversed = false; //Q2
    public static final boolean kRearRightEncoderReversed = false; //Q3
    public static final boolean kFrontRightEncoderReversed = false; //Q4

    public static final Rotation2d kFrontLeftEncoderOffset = new Rotation2d(Math.toRadians(277.559)); //Q1
    public static final Rotation2d kRearLeftEncoderOffset = new Rotation2d(Math.toRadians(126.826)); //Q2
    public static final Rotation2d kRearRightEncoderOffset = new Rotation2d(Math.toRadians(250.137)); //Q3
    public static final Rotation2d kFrontRightEncoderOffset = new Rotation2d(Math.toRadians(45.703)); //Q4

    public static final double kTrackWidth = 0.431; // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.681; // Distance between front and back wheels on robot

    public static final Vector<N3> stateStdDeviations = VecBuilder.fill(0.005, 0.005, Math.toRadians(1));
    public static final Vector<N3> visionStdDeviations = VecBuilder.fill(0.050, 0.050, Math.toRadians(5));

    public static final SwerveDriveKinematics kDriveKinematics =
      new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), //Q1
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2), //Q2
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), //Q3
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2)); //Q4

    public static final boolean kGyroReversed = false;

    // If the aggregate velocity of the swerve modules is beneath this threshold, the robot is considered to be at rest,
    // in which case housekeeping such as re-synchronizing the turning encoders may take place.
    public static final double kAggregateSpeedThresholdMetersPerSecond = 0.01;

    // Note that SwerveModuleConstants.kMaxSpeedMedersPerSecond may saturate if this is set too high, in combination with
    // SwerveModuleConstants.kMaxAngularSpeedRadiansPerSecond.
    public static final double kMaxSpeedMetersPerSecond = 5.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 7.5;
    public static final double kMaxDecelerationMetersPerSecondSquared = 15.0;

    public static final double kMaxAngularSpeedRadiansPerSecond = 3.0 * Math.PI;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = 1.5 * Math.PI;
    public static final double kMaxAngularDecelerationRadiansPerSecondSquared = 3.0 * Math.PI;

    public static final TrapezoidalConstraint kVelocityProfile = new TrapezoidalConstraint(
      kMaxSpeedMetersPerSecond,
      kMaxAccelerationMetersPerSecondSquared,
      kMaxDecelerationMetersPerSecondSquared
    );

    public static final TrapezoidalConstraint kAngularVelocityProfile = new TrapezoidalConstraint(
      kMaxAngularSpeedRadiansPerSecond,
      kMaxAngularAccelerationRadiansPerSecondSquared,
      kMaxAngularDecelerationRadiansPerSecondSquared
    );
  }

  public static final class SwerveModuleConstants {
    public static final double kMaxSpeedMetersPerSecond = DriveConstants.kMaxSpeedMetersPerSecond * 2.0;

    public static final double kAbsoluteEncoderCPR = 4096.0;
    public static final double kWheelDiameterMeters = 0.09525;

    public static final double kDriveGearRatio = 6.75;
    public static final double kTurningGearRatio = 150.0 / 7.0;

    // The native position units are motor rotations, but we want meters.
    public static final double kDrivePositionConversionFactor =
      (SwerveModuleConstants.kWheelDiameterMeters * Math.PI)
      / SwerveModuleConstants.kDriveGearRatio;
    // The native velocity units are motor rotations [aka revolutions] per minute (RPM),
    // but we want meters per second.
    public static final double kDriveVelocityConversionFactor =
      kDrivePositionConversionFactor
      / 60.0 /* s */;

    // The native position units are motor rotations, but we want radians.
    public static final double kTurningPositionConversionFactor =
      Units.rotationsToRadians(1.0 / SwerveModuleConstants.kTurningGearRatio);
    // The native velocity units are motor rotations [aka revolutions] per minute (RPM), but we want radians per second.
    public static final double kTurningVelocityConversionFactor =
      kTurningPositionConversionFactor
      / 60.0 /* s */;

    public static final PID kDrivePID = new PID(0.5, 0.0, 0.0);
    public static final PID kTurningPID = new PID(0.5, 0.0, 0.0);

    public static final int kDriveMotorCurrentLimit = 40;
    public static final int kTurningMotorCurrentLimit = 20;

    public static final double kDriveMotorRampRate = 0.25;
    public static final double kTurningMotorRampRate = 0.25;

    public static final long kValueCacheTtlMicroseconds = 15;
  }

  public static class ShooterConstants {
    //TODO: Assign real CAN ID's for the 2 flywheels
    public static final int kLeftFlywheelID = 0;
    public static final int kRigthFlywheelID = 0;

    public static final int kShooterCurrentLimit = 40;

    public static final double kFlywheelEncoderCPR = 42.0;
    public static final double kAbsoluteEncoderCPR = 4096.0;
    //TODO: Find the flywheel diameter
    public static final double kFlywheelDiameterMeters = 0.09525;
    //TODO: Find gear ratio
    public static final double kFlywheelGearRatio = 6.75;

    public static final double kFlywheelEncoderDistancePerPulse =
        (kFlywheelDiameterMeters * Math.PI) / (kFlywheelEncoderCPR * kFlywheelGearRatio);
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1.2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
  }
  
  public static final class VisionConstants {
    public static final double kTargetingTolerance = Units.degreesToRadians(3);
    public static final double kLimelightMountDegrees = -5;
    public static final double kLimelightLensHeightInches = 16.25;
    public static final double kNoteHeightInches = 2.0;
    public static final double kTargetCoefficient = 30;
    public static final int kRelayPort = 0;
  }

  public static class FieldConstants {
    public static final double kFieldLength = 4.0;
    public static final double kFieldWidth = 3.0;
    public static final double kMaxX = 16.54;
  }

  public static class PhotonVisionConstants {
    public static final String kCameraName1 = "camera1";
    public static final String kCameraName2 = "camera2";
    public static final Transform3d kRobotToCamera1Transform = new Transform3d(
      0.273, -0.057, 0.270,
      new Rotation3d(0.0, Units.degreesToRadians(-19.5), 0.0));
    public static final Transform3d kRobotToCamera2Transform = new Transform3d(
      0.191, 0.016, 0.349,
      new Rotation3d(Units.degreesToRadians(-21.9), 0.0, Units.degreesToRadians(100.0)));
  }
}
