// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Map;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
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
  public static final boolean kBurnMotors = false;

  // Tunable constants are disabled unless this is set to true.
  // Intended to remain false in committed code.
  public static final boolean kEnableTuning = false;

  public static final class DriveConstants {
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

    public static final boolean kFrontLeftDriveReversed = true; //Q1
    public static final boolean kRearLeftDriveReversed = true; //Q2
    public static final boolean kRearRightDriveReversed = false; //Q3
    public static final boolean kFrontRightDriveReversed = true; //Q4

    public static final boolean kFrontLeftTurningMotorReversed = true; //Q1
    public static final boolean kRearLeftTurningMotorReversed = false; //Q2
    public static final boolean kRearRightTurningMotorReversed = true; //Q3
    public static final boolean kFrontRightTurningMotorReversed = true; //Q4

    public static final boolean kFrontLeftEncoderReversed = false; //Q1
    public static final boolean kRearLeftEncoderReversed = false; //Q2
    public static final boolean kRearRightEncoderReversed = false; //Q3
    public static final boolean kFrontRightEncoderReversed = false; //Q4

    public static final Rotation2d kFrontLeftEncoderOffset = new Rotation2d(Units.rotationsToRadians(0.661133)); //Q1
    public static final Rotation2d kRearLeftEncoderOffset = new Rotation2d(Units.rotationsToRadians(0.584229)); //Q2
    public static final Rotation2d kRearRightEncoderOffset = new Rotation2d(Units.rotationsToRadians(0.115234)); //Q3
    public static final Rotation2d kFrontRightEncoderOffset = new Rotation2d(Units.rotationsToRadians(0.056641)); //Q4

    public static final boolean kSquareInputs = false;

    public static final double kTrackWidth = 0.629; // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.629; // Distance between front and back wheels on robot

    public static final Vector<N3> stateStdDeviations =
      VecBuilder.fill(0.005, 0.005, Math.toRadians(1.0));
    public static final Vector<N3> visionStdDeviations =
      VecBuilder.fill(0.050, 0.050, Math.toRadians(5.0));

    public static final SwerveDriveKinematics kDriveKinematics =
      new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0), //Q1
        new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0), //Q2
        new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0), //Q3
        new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0)); //Q4

    public static final boolean kGyroReversed = false;

    // If the aggregate velocity of the swerve modules is beneath this threshold, the robot is considered to be at rest,
    // in which case housekeeping such as re-synchronizing the turning encoders may take place.
    public static final double kAggregateSpeedThresholdMetersPerSecond = 0.01;

    // Note that SwerveModuleConstants.kMaxSpeedMedersPerSecond may saturate if this is set too high, in combination with
    // SwerveModuleConstants.kMaxAngularSpeedRadiansPerSecond.
    public static final double kMaxSpeedMetersPerSecond = 5.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 10.0;
    public static final double kMaxDecelerationMetersPerSecondSquared = 30.0;

    public static final double kMaxAngularSpeedRadiansPerSecond = 3.0 * Math.PI;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = 3.0 * Math.PI;
    public static final double kMaxAngularDecelerationRadiansPerSecondSquared = 9.0 * Math.PI;

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
    public static final int kAutoPIDFSlotID = 0;
    public static final int kTeleopPIDFSlotID = 1;
    public static final int kDefaultPIDFSlotID = kAutoPIDFSlotID;

    public static final double kMaxSpeedMetersPerSecond = DriveConstants.kMaxSpeedMetersPerSecond * 2.0;

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

    // The NEO's relative encoder can return spurious results during initialization, and there is
    // no synchronous API for setting the encoder value. This presents a conundrum regarding how to
    // get valid/stable output early on. Our heuristic strategy is to set the encoder to zero, and
    // keep reading the encoder value until it is acceptably close to zero. This value can be quite
    // low, but there is the hypothetical danger of encoder jitter causing an infinite loop as the
    // tolerance approches zero.
    public static final double kTurningEncoderStabilizeToleranceRadians =
      Units.degreesToRadians(1.0);

    public static final PIDF kAutoDrivePIDF = new PIDF(0.5, 0.002, 0.0);
    public static final PIDF kAutoTurningPIDF = new PIDF(0.5, 0.0, 0.0);

    public static final PIDF kTeleopDrivePIDF = new PIDF(0.375, 0.0, 0.0, 0.25);
    // Use SwerveModule.tunableTeleopTurningPIDF rather than directly using this field.
    public static final PIDF kTeleopTurningPIDF = new PIDF(0.375, 0.0, 0.0);

    public static final int kDriveMotorCurrentLimit = 40;
    public static final int kTurningMotorCurrentLimit = 20;

    public static final double kDriveMotorRampRate = 0.25;
    public static final double kTurningMotorRampRate = 0.25;

    public static final long kValueCacheTtlMicroseconds = 15;
    }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
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

    public static final int kLeftJoyXAxis = 0;
    public static final int kLeftJoyYAxis = 1;
    public static final int kLeftTriggerAxis = 2;
    public static final int kRightTriggerAxis = 3;
    public static final int kRightJoyXAxis = 4;
    public static final int kRightJoyYAxis = 5;

    public static final int kZeroGyro = kStart;
    public static final int kJoystickTargetNoteButton = kA;
    public static final int kJoystickTargetSpeakerButton = kY;
    public static final int kEjectButton = kRightBumper;
    public static final int kShootButton = kRightBumper;

    public static final int kIntakeOn = kX;
    public static final int kIntakeOff = kB;
    public static final int kIntakeReverse = kA;
    
    public static final double kDebounceSeconds = 0.01;

    public static final double kJoystickDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.0;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final PIDConstants kTranslationHolonomicPID= new PIDConstants(5.0, 0.0, 0.0);
    public static final PIDConstants kRotationHolonomicPID= new PIDConstants(5.0, 0.0, 0.0);

    public static final double kDriveBaseRadius = 0.4;
  }
  
  public static final class LimelightConstants {
    public static final boolean kEnable = true;
    public static final double kLimelightMountDegrees = -20.0;
    public static final double kLimelightLensHeightInches = 16.25;
    public static final double kNoteHeightInches = 2.0;
    public static final int kRelayPort = 3;
    public static final int kFilterSize = 7;
  }
  
  public static final class NoteConstants {
    public static final double kAngularTolerance = Units.degreesToRadians(5.0);
    // Use JoystickTargetNote.targetTurningPIDF rather than directly using this field.
    public static final PIDF kTurningPIDF = new PIDF(3.0, 0.0, 0.0, 0.2);
  }

  public static final class PhotonVisionConstants {
    public static final boolean kEnable = true;
    public static final String kCameraName1 = "camera1";
    public static final String kCameraName2 = "camera2";
    public static final Transform3d kRobotToCamera1Transform = new Transform3d(
      0.196, -0.238, 0.26,
      new Rotation3d(0.0, Units.degreesToRadians(-21.0), 0.0));
    public static final Transform3d kRobotToCamera2Transform = new Transform3d(
      0.196, 0.238, 0.26,
      new Rotation3d(0.0, Units.degreesToRadians(-20.5), 0.0));
  }

  public static final class SpeakerConstants {
    public static final double kMinPossibleShootingDistance = 2.39;
    public static final double kMinGoodShootingDistance = 2.54;
    public static final double kMaxGoodShootingDistance = 3.56;
    public static final double kMaxPossibleShootingDistance = 4.01;

    public static final double kAngularTolerance = Units.degreesToRadians(1.0);
    // Use JoystickTargetSpeaker.targetTurningPIDF rather than directly using this field.
    public static final PIDF kTurningPIDF = NoteConstants.kTurningPIDF;
  }

  public static final class FieldConstants {
    private static final AprilTagFieldLayout loadTransformedAprilTagFieldLayout() {
      final AprilTagFieldLayout rawLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
      Map<Integer, Rotation3d> rotations = Map.of(
        // In order to adjust an AprilTag's rotation, specify non-zero roll/pitch/yaw, as viewed
        // from perspective of the field XYZ axes. For example, if the blue speaker AprilTag is
        // tilted upward by 2 degrees:
        //
        //   7, new Rotation3d(0.0, Units.degreesToRadians(-2.0), 0.0)
        //
        // The same upward tilt for the red speaker AprilTag would be corrected as such:
        //
        //   4, new Rotation3d(0.0, Units.degreesToRadians(2.0), 0.0)
      );
      List<AprilTag> aprilTags =
        rawLayout.getTags().stream()
        .map(aprilTag -> {
          AprilTag rotatedAprilTag;
          if (rotations.containsKey(aprilTag.ID)) {
            rotatedAprilTag =
              new AprilTag(aprilTag.ID, aprilTag.pose.rotateBy(rotations.get(aprilTag.ID)));
          } else {
            rotatedAprilTag = aprilTag;
          }
          return rotatedAprilTag;
        })
        .toList();
      return new AprilTagFieldLayout(
        aprilTags, rawLayout.getFieldLength(), rawLayout.getFieldWidth());
    }
    public static final AprilTagFieldLayout kAprilTagFieldLayout =
      loadTransformedAprilTagFieldLayout();

    public static final double kMaxX = kAprilTagFieldLayout.getFieldLength();

    private static Translation2d getAprilTagTranslation(AprilTagFieldLayout fieldLayout, int aprilTagID) {
      for (AprilTag aprilTag : fieldLayout.getTags()) {
        if (aprilTag.ID == aprilTagID) {
          return new Translation2d(aprilTag.pose.getX(), aprilTag.pose.getY());
        }
      }
      assert(false);
      return new Translation2d();
    }
    public static final Translation2d kBlueSpeaker = getAprilTagTranslation(kAprilTagFieldLayout, 7);
    public static final Translation2d kRedSpeaker = getAprilTagTranslation(kAprilTagFieldLayout, 4);
  }

  public static final class IntakeConstants {
    public static final int kTopMotorID = 13;
    public static final int kBottomMotorID = 14;
    public static final int kCurrentLimit = 20;
    public static final boolean kTopMotorReversed = false;
    public static final boolean kBottomMotorReversed = false;
    public static final double kTopWheelDiameterMeters = Units.inchesToMeters(2.0);
    public static final double kBottomWheelDiameterMeters = Units.inchesToMeters(1.0);
    public static final double kTopGearRatio = 3.0;
    public static final double kBottomGearRatio = 3.0;
    // VelocityCoversionFactors convert between revolutions per minute and m/s
    public static final double kTopVelocityConversionFactor =
      (kTopWheelDiameterMeters * Math.PI)
      / (kTopGearRatio * 60.0);
    public static final double kBottomVelocityConversionFactor =
      (kBottomWheelDiameterMeters * Math.PI)
      / (kBottomGearRatio * 60.0);
    public static final PIDF kTopPIDF = new PIDF(0.1, 0.0, 0.0, 0.2075);
    public static final PIDF kBottomPIDF = new PIDF(0.2, 0.0, 0.0, 0.405);
    public static final double kSpeed = 2.5;
  }

  public static final class ChamberConstants {
    public static final int kLeftMotorID = 15;
    public static final int kRightMotorID = 16;
    public static final int kSensorID = 5;
    public static final int kCurrentLimit = 20;
    public static final boolean kLeftMotorReversed = false;
    public static final boolean kRightMotorReversed = true;
    // Sensor sense may be reversed (false when note is detected), in which case this should be set
    // to false.
    public static final boolean kSensorNoteDetectedValue = true;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3.0);
    public static final double kGearRatio = 3.0 / 1.0;
    // VelocityCoversionFactors convert between revolutions per minute and m/s
    public static final double kChamberVelocityConversionFactor =
      (kWheelDiameterMeters * Math.PI)
      / kGearRatio / 60.0;
    public static final PIDF kLeftPIDF = new PIDF(0.1, 0.0, 0.0, 0.1525);
    public static final PIDF kRightPIDF = new PIDF(0.1, 0.0, 0.0, 0.15);
    public static final double kIntakingSpeed = IntakeConstants.kSpeed;
    public static final double kShootingSpeed = 6.5;
    public static final double kClearingTimeSeconds = 1.0;
    public static final double kAligningSpeed = -1.0;
  }

  public static final class ShooterConstants {
    public static final int kLeftMotorID = 17;
    public static final int kRightMotorID = 18;
    public static final int kCurrentLimit = 40;
    public static final boolean kLeftMotorReversed = false;
    public static final boolean kRightMotorReversed = true;

    public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);
    public static final double kGearRatio = 1.0;

    // The native position units are motor rotations, but we want meters.
    public static final double kPositionConversionFactor =
      (ShooterConstants.kWheelDiameterMeters * Math.PI)
      / ShooterConstants.kGearRatio;
    // The native velocity units are motor rotations [aka revolutions] per minute (RPM),
    // but we want meters per second.
    public static final double kVelocityConversionFactor =
      kPositionConversionFactor
      / 60.0 /* s */;

    public static final PIDF kPIDF = new PIDF(0.08, 0.0, 3, 0.1);
    public static final double kIdleSpeed = 15.0;
    public static final double kShootingSpeed = 25.0;

    public static final double kSpinSpeed = 0.0;

    public static final double kShootingTolerance = 0.05;
  }

  public static final class ClimberConstants {
    public static final boolean kEnable = false;
    public static final int kLeftMotorID = 19;
    public static final int kRightMotorID = 20;
    public static final int kCurrentLimit = 40;
    public static final boolean kLeftMotorReversed = false;
    public static final boolean kRightMotorReversed = false;
    public static final PIDF kPIDF = new PIDF(0.5, 0.0, 0.0);
    public static final double kGearRatio = 3.0;
    // With no gear reduction each motor rotation moves the climber 0.012 meters
    public static final double kPositionConversionFactor = 0.012 / kGearRatio;
    public static final double kVelocityConversionFactor = kPositionConversionFactor / 60.0;
    public static final double kMinRetractedLength = 0.0;
    public static final double kMaxExtendedLength = 0.304;
    public static final double kMaxSpeed = 0.330;
    public static final double kInitializingSpeed = -0.03;
    public static final double kConstrainedRange = 0.0762;
  }
}
