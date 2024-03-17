package frc.robot.utilities;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;

public class TargetSpeaker {
  private final Translation2d m_speaker;

  public TargetSpeaker() {
    m_speaker = getSpeaker(getAlliance());
  }

  private static Alliance getAlliance() {
    Optional<Alliance> allianceOpt = DriverStation.getAlliance();
    Alliance alliance = allianceOpt.isPresent() ? allianceOpt.get() : Alliance.Blue;
    return alliance;
  }

  private static Translation2d getSpeaker(Alliance alliance) {
    Translation2d speaker;
    switch (alliance) {
      default: assert false;
      case Blue: {
        speaker = FieldConstants.kBlueSpeaker;
        break;
      }
      case Red: {
        speaker = FieldConstants.kRedSpeaker;
        break;
      }
    }
    return speaker;
  }

  private Translation2d getRobotToSpeaker(Pose2d robotPose) {
    Translation2d robotToSpeaker = m_speaker.minus(robotPose.getTranslation());
    return robotToSpeaker;
  }

  public Rotation2d getRotationDeviation(Pose2d robotPose) {
    Rotation2d currentRotation = robotPose.getRotation();
    Rotation2d desiredRotation = getRobotToSpeaker(robotPose).getAngle();
    Rotation2d rotationDeviation = currentRotation.minus(desiredRotation);
    return rotationDeviation;
  }

  public double getSpeakerDistance(Pose2d robotPose) {
    Translation2d robotToSpeaker = getRobotToSpeaker(robotPose);
    double speakerDistance = robotToSpeaker.getNorm();
    return speakerDistance;
  }
}
