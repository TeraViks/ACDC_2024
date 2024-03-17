package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SpeakerConstants;
import frc.robot.subsystems.Chamber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.PIDF;
import frc.robot.utilities.ShooterInterp;
import frc.robot.utilities.TargetSpeaker;
import frc.robot.utilities.TunablePIDF;

public class JoystickTargetSpeaker extends Command {
  private static final TunablePIDF targetTurningPIDF =
    new TunablePIDF("Speaker.turningPIDF", SpeakerConstants.kTurningPIDF);
  private final DriveSubsystem m_drive;
  private final Chamber m_chamber;
  private final Shooter m_shooter;
  private final GenericHID m_driverController;
  private final Supplier<Double> m_xVelocitySupplier;
  private final Supplier<Double> m_yVelocitySupplier;
  private final TargetSpeaker m_targetSpeaker;

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
      Supplier<Double> xVelocitySupplier, Supplier<Double> yVelocitySupplier,
      TargetSpeaker targetSpeaker) {
    m_drive = drive;
    m_chamber = chamber;
    m_shooter = shooter;
    m_driverController = driverController;
    m_xVelocitySupplier = xVelocitySupplier;
    m_yVelocitySupplier = yVelocitySupplier;

    m_targetSpeaker = targetSpeaker;

    m_thetaController.setTolerance(SpeakerConstants.kAngularTolerance);
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drive, chamber, shooter);
  }

  @Override
  public void initialize() {
    Pose2d robotPose = m_drive.getPose();
    m_thetaController.reset(
      m_targetSpeaker.getRotationDeviation(robotPose).getRadians(),
      m_drive.getAngularVelocity());
  }

  @Override
  public void execute() {
    Pose2d robotPose = m_drive.getPose();
    Rotation2d rotationDeviation = m_targetSpeaker.getRotationDeviation(robotPose);

    // Turn toward speaker.
    updateConstants();
    double thetaVelocity = m_thetaController.calculate(rotationDeviation.getRadians());
    m_drive.drive(
      m_xVelocitySupplier.get(),
      m_yVelocitySupplier.get(),
      thetaVelocity,
      true
    );

    double speakerDistance = m_targetSpeaker.getSpeakerDistance(robotPose);
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
    Rotation2d rotationDeviation = m_targetSpeaker.getRotationDeviation(robotPose);
    return (Math.abs(rotationDeviation.getRadians()) <= SpeakerConstants.kAngularTolerance);
  }

  @Override
  public void end(boolean interrupted) {
    // TODO: Consider what to do about shooter revving rather than idling.
    m_driverController.setRumble(RumbleType.kBothRumble, 0);
  }
}