package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.TargetConstants;
import frc.robot.PIDF;
import frc.robot.subsystems.Chamber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterInterp;
import frc.robot.subsystems.SwerveModule;

public class JoystickTargetSpeaker extends Command {
  private final DriveSubsystem m_drive;
  private final Chamber m_chamber;
  private final Shooter m_shooter;
  private final GenericHID m_driverController;
  private final Supplier<Double> m_xVelocitySupplier;
  private final Supplier<Double> m_yVelocitySupplier;
  private final Translation2d m_speaker;

  private ProfiledPIDController m_thetaController = new ProfiledPIDController(
    SwerveModule.tunableTeleopTurningPIDF.get().p(),
    SwerveModule.tunableTeleopTurningPIDF.get().i(),
    SwerveModule.tunableTeleopTurningPIDF.get().i(),
    new TrapezoidProfile.Constraints(
      DriveConstants.kMaxAngularSpeedRadiansPerSecond,
      DriveConstants.kMaxAngularAccelerationRadiansPerSecondSquared));

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

    addRequirements(drive, chamber, shooter);
  }

  @Override
  public void initialize() {}

  private Translation2d computeSpeakerTranslation(Pose2d robotPose) {
    return robotPose.getTranslation().minus(m_speaker);
  }

  @Override
  public void execute() {
    Pose2d robotPose = m_drive.getPose();
    Translation2d speakerTranslation = computeSpeakerTranslation(robotPose);
    double speakerDistance = speakerTranslation.getNorm();
    Rotation2d speakerAngle = speakerTranslation.getAngle();

    // Turn toward speaker.
    updateConstants();
    double thetaVelocity =
      m_thetaController.calculate(speakerAngle.getRadians() *
        JoystickTargetNote.targetAngularVelocityCoefficient.get());
    m_drive.drive(
      m_xVelocitySupplier.get(),
      m_yVelocitySupplier.get(),
      thetaVelocity,
      true
    );

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
    if (speakerDistance <= TargetConstants.kMaxGoodShootingDistance) {
      if (speakerDistance < TargetConstants.kMinGoodShootingDistance) {
        if (speakerDistance < TargetConstants.kMinPossibleShootingDistance) {
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
    } else if (speakerDistance <= TargetConstants.kMaxPossibleShootingDistance) {
      // Farther than ideal.
      m_driverController.setRumble(RumbleType.kLeftRumble, 0);
      m_driverController.setRumble(RumbleType.kRightRumble, 1);
    } else {
      // Too far.
      m_driverController.setRumble(RumbleType.kBothRumble, 0);
    }
  }

  private void updateConstants() {
    if (SwerveModule.tunableTeleopTurningPIDF.hasChanged()) {
      PIDF pid = SwerveModule.tunableTeleopTurningPIDF.get();
      m_thetaController.setPID(pid.p(), pid.i(), pid.d());
    }
  }

  @Override
  public void end(boolean interrupted) {
    // TODO: Consider what to do about shooter revving rather than idling.
    m_driverController.setRumble(RumbleType.kBothRumble, 0);
  }
}