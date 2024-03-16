package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.NoteConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SpeakerConstants;
import frc.robot.PIDF;
import frc.robot.ShooterInterp;
import frc.robot.TunablePIDF;
import frc.robot.subsystems.Chamber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Shooter;

public class JoystickTargetSpeaker extends Command {
  public static TunablePIDF targetTurningPIDF =
    new TunablePIDF("Speaker.turningPIDF", SpeakerConstants.kTurningPIDF);
  private final DriveSubsystem m_drive;
  private final Chamber m_chamber;
  private final Shooter m_shooter;
  private final GenericHID m_driverController;
  private final Supplier<Double> m_xVelocitySupplier;
  private final Supplier<Double> m_yVelocitySupplier;
  private final Translation2d m_speaker;

  private ProfiledPIDController m_thetaController = new ProfiledPIDController(
    JoystickTargetSpeaker.targetTurningPIDF.get().p(),
    JoystickTargetSpeaker.targetTurningPIDF.get().i(),
    JoystickTargetSpeaker.targetTurningPIDF.get().d(),
    new TrapezoidProfile.Constraints(
      DriveConstants.kMaxAngularSpeedRadiansPerSecond,
      DriveConstants.kMaxAngularAccelerationRadiansPerSecondSquared),
    Constants.kDt);

  public JoystickTargetSpeaker(DriveSubsystem drive, Chamber chamber, Shooter shooter,
      GenericHID driverController,
      Supplier<Double> xVelocitySupplier, Supplier<Double> yVelocitySupplier) {
    m_drive = drive;
    m_chamber = chamber;
    m_shooter = shooter;
    m_driverController = driverController;
    m_xVelocitySupplier = xVelocitySupplier;
    m_yVelocitySupplier = yVelocitySupplier;

    Optional<Alliance> allianceOpt = DriverStation.getAlliance();
    Alliance alliance = allianceOpt.isPresent() ? allianceOpt.get() : Alliance.Blue;
    switch (alliance) {
      default: assert false;
      case Blue: {
        m_speaker = FieldConstants.kBlueSpeaker;
        break;
      }
      case Red: {
        m_speaker = FieldConstants.kRedSpeaker;
        break;
      }
    }

    m_thetaController.setTolerance(SpeakerConstants.kAngularTolerance);
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drive, chamber, shooter);
  }

  private Translation2d getRobotToSpeaker(Pose2d robotPose) {
    Translation2d robotToSpeaker = m_speaker.minus(robotPose.getTranslation());
    return robotToSpeaker;
  }

  private Rotation2d getRotationDeviation(Pose2d robotPose, Translation2d robotToSpeaker) {
    Rotation2d currentRotation = robotPose.getRotation();
    Rotation2d desiredRotation = robotToSpeaker.getAngle();
    Rotation2d rotationDeviation = currentRotation.minus(desiredRotation);
    return rotationDeviation;
  }

  private double getSpeakerDistance(Translation2d robotToSpeaker) {
    double speakerDistance = robotToSpeaker.getNorm();
    return speakerDistance;
  }

  @Override
  public void initialize() {
    Pose2d robotPose = m_drive.getPose();
    Translation2d robotToSpeaker = getRobotToSpeaker(robotPose);
    m_thetaController.reset(
      getRotationDeviation(robotPose, robotToSpeaker).getRadians(),
      m_drive.getAngularVelocity());
  }

  @Override
  public void execute() {
    Pose2d robotPose = m_drive.getPose();
    Translation2d robotToSpeaker = getRobotToSpeaker(robotPose);
    Rotation2d rotationDeviation = getRotationDeviation(robotPose, robotToSpeaker);

    // Turn toward speaker.
    updateConstants();
    double thetaVelocity = m_thetaController.calculate(rotationDeviation.getRadians());
    m_drive.drive(
      m_xVelocitySupplier.get(),
      m_yVelocitySupplier.get(),
      thetaVelocity,
      true
    );

    double speakerDistance = getSpeakerDistance(robotToSpeaker);
    revShooter(speakerDistance);
    rumbleDriverController(speakerDistance);
  }

  private void revShooter(double speakerDistance) {
    if (m_chamber.isNoteChambered()) {
      double shooterSpeed = ShooterInterp.distanceToSpeed(speakerDistance);
      m_shooter.revShooter(shooterSpeed);
    }
  }

  private void rumbleDriverController(double speakerDistance) {
    // There are five cases, which in the worst case requires three conditional branches to
    // determine (ceiling of log_2(5)). Structure conditionals such that the common case of
    // approaching from afar requires only two branches to select.
    if (speakerDistance <= SpeakerConstants.kMaxGoodShootingDistance) {
      if (speakerDistance < SpeakerConstants.kMinGoodShootingDistance) {
        if (speakerDistance < SpeakerConstants.kMinPossibleShootingDistance) {
          // Too near.
          m_driverController.setRumble(RumbleType.kBothRumble, 0);
        } else {
          // Nearer than ideal.
          m_driverController.setRumble(RumbleType.kLeftRumble, 1);
          m_driverController.setRumble(RumbleType.kRightRumble, 0);
        }
      } else {
        // Ideal distance.
        m_driverController.setRumble(RumbleType.kBothRumble, 1);
      }
    } else if (speakerDistance <= SpeakerConstants.kMaxPossibleShootingDistance) {
      // Farther than ideal.
      m_driverController.setRumble(RumbleType.kLeftRumble, 0);
      m_driverController.setRumble(RumbleType.kRightRumble, 1);
    } else {
      // Too far.
      m_driverController.setRumble(RumbleType.kBothRumble, 0);
    }
  }

  private void updateConstants() {
    if (targetTurningPIDF.hasChanged()) {
      PIDF pidf = targetTurningPIDF.get();
      m_thetaController.setPID(pidf.p(), pidf.i(), pidf.d());
    }
  }

  public boolean isFacingSpeaker() {
    Pose2d robotPose = m_drive.getPose();
    Translation2d robotToSpeaker = getRobotToSpeaker(robotPose);
    Rotation2d rotationDeviation = getRotationDeviation(robotPose, robotToSpeaker);
    return (Math.abs(rotationDeviation.getDegrees()) <= NoteConstants.kAngularTolerance);
  }

  @Override
  public void end(boolean interrupted) {
    // TODO: Consider what to do about shooter revving rather than idling.
    m_driverController.setRumble(RumbleType.kBothRumble, 0);
  }
}