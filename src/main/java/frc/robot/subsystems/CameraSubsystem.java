// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhotonVisionConstants;

public class CameraSubsystem extends SubsystemBase {
  PhotonCamera m_camera1;
  PhotonCamera m_camera2;
  PhotonPoseEstimator m_photonPoseEstimatorCam1;
  PhotonPoseEstimator m_photonPoseEstimatorCam2;
  PhotonPipelineResult m_resultCam1;
  PhotonPipelineResult m_resultCam2;
  Optional<PhotonTrackedTarget> m_lowestAmbiguityTarget;

  AprilTagFieldLayout m_aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  public CameraSubsystem(String Camera1Name, String Camera2Name) {
    m_camera1 = new PhotonCamera(Camera1Name);
    m_camera2 = new PhotonCamera(Camera2Name);

    m_photonPoseEstimatorCam1 = 
      new PhotonPoseEstimator(
        m_aprilTagFieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        m_camera1,
        PhotonVisionConstants.kRobotToCamera1Transform
      );

    m_photonPoseEstimatorCam2 =
      new PhotonPoseEstimator(
        m_aprilTagFieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        m_camera2,
        PhotonVisionConstants.kRobotToCamera2Transform
      );

    m_photonPoseEstimatorCam1.setTagModel(TargetModel.kAprilTag36h11);
    m_photonPoseEstimatorCam2.setTagModel(TargetModel.kAprilTag36h11);
  }

  public ArrayList<Optional<EstimatedRobotPose>> getFieldRelativePoseEstimators() {
    ArrayList<Optional<EstimatedRobotPose>> estimatorList = new ArrayList<Optional<EstimatedRobotPose>>();
    estimatorList.add(m_photonPoseEstimatorCam1.update(m_resultCam1));
    estimatorList.add(m_photonPoseEstimatorCam2.update(m_resultCam2));
    return estimatorList;
  }

  /* Given previous and current results from getFieldRelativePoseEstimators(), exclude stale
   * estimates, i.e. those present in previous results, selecting only fresh estimates, if any. */
  public ArrayList<EstimatedRobotPose> selectFreshEstimates(
      ArrayList<Optional<EstimatedRobotPose>> prev,
      ArrayList<Optional<EstimatedRobotPose>> cur) {
    ArrayList<EstimatedRobotPose> freshEstimates = new ArrayList<>();

    if (prev.size() == 0) {
      // No prev estimates, therefore all cur estimates are fresh.
      for (Optional<EstimatedRobotPose> curEstimateOpt : cur) {
        if (curEstimateOpt.isPresent()) {
          freshEstimates.add(curEstimateOpt.get());
        }
      }
    } else {
      assert(prev.size() == cur.size());
      Iterator<Optional<EstimatedRobotPose>> prevIterator = prev.iterator();
      Iterator<Optional<EstimatedRobotPose>> curIterator = cur.iterator();

      // Iterate over prev and cur in lockstep.
      while (prevIterator.hasNext() && curIterator.hasNext()) {
        Optional<EstimatedRobotPose> prevEstimateOpt = prevIterator.next();
        Optional<EstimatedRobotPose> curEstimateOpt = curIterator.next();

        if (curEstimateOpt.isPresent()) {
          EstimatedRobotPose curEstimate = curEstimateOpt.get();
          if (prevEstimateOpt.isPresent()) {
            EstimatedRobotPose prevEstimate = prevEstimateOpt.get();
            if (curEstimate.timestampSeconds > prevEstimate.timestampSeconds) {
              // Newer timestamp, therefore fresh.
              freshEstimates.add(curEstimate);
            }
          } else {
            // Missing in prev, therefore fresh.
            freshEstimates.add(curEstimate);
          }
        }
      }
    }

    return freshEstimates;
  }

  @Override
  public void periodic() {
    m_resultCam1 = m_camera1.getLatestResult();
    m_resultCam2 = m_camera2.getLatestResult();
  }
}
