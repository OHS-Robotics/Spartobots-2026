// Copyright (c) 2021-2026 Littleton Robotics
// Copyright (c) 2026 Team 4687 Spartobots
//
// SPDX-License-Identifier: BSD-3-Clause

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
  private final PhotonPoseEstimator poseEstimator;
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;
  private final Supplier<Pose2d> robotPoseSupplier;

  /**
   * Creates a new VisionIOPhotonVision.
   *
   * @param name The configured name of the camera.
   * @param robotToCamera The 3D position of the camera relative to the robot.
   */
  public VisionIOPhotonVision(
      String name,
      Transform3d robotToCamera,
      int pipelineIndex,
      Supplier<Pose2d> robotPoseSupplier) {
    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
    this.robotPoseSupplier = robotPoseSupplier;
    poseEstimator = new PhotonPoseEstimator(aprilTagLayout, robotToCamera);
    camera.setPipelineIndex(pipelineIndex);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = false;
    inputs.processedResultCount = 0;
    inputs.detectedTargetCount = 0;
    inputs.latestResultTimestampSeconds = Double.NaN;
    inputs.latestTargetObservation = new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
    inputs.poseObservations = new PoseObservation[0];
    inputs.tagIds = new int[0];

    final List<PhotonPipelineResult> unreadResults;
    try {
      inputs.connected = camera.isConnected();
      unreadResults = camera.getAllUnreadResults();
    } catch (RuntimeException exception) {
      return;
    }

    // Read new camera observations
    inputs.processedResultCount = unreadResults.size();
    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    for (var result : unreadResults) {
      inputs.detectedTargetCount += result.targets.size();
      inputs.latestResultTimestampSeconds = result.getTimestampSeconds();

      // Update latest target observation
      if (result.hasTargets()) {
        inputs.latestTargetObservation =
            new TargetObservation(
                Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
      } else {
        inputs.latestTargetObservation = new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
      }

      if (!result.hasTargets()) {
        continue;
      }

      poseEstimator.resetHeadingData(
          result.getTimestampSeconds(), robotPoseSupplier.get().getRotation());

      EstimatedRobotPose estimatedRobotPose;
      double ambiguity;
      int tagCount;
      double averageTagDistance;

      // Add pose observation
      if (result.multitagResult.isPresent()) { // Multitag result
        var multitagResult = result.multitagResult.get();
        var estimate = poseEstimator.estimateCoprocMultiTagPose(result);
        if (estimate.isEmpty()) {
          continue;
        }
        estimatedRobotPose = estimate.get();

        // Calculate average tag distance
        double totalTagDistance = 0.0;
        for (var target : result.targets) {
          totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
        }

        // Add tag IDs
        tagIds.addAll(multitagResult.fiducialIDsUsed);
        ambiguity = multitagResult.estimatedPose.ambiguity;
        tagCount = multitagResult.fiducialIDsUsed.size();
        averageTagDistance = totalTagDistance / result.targets.size();

      } else if (!result.targets.isEmpty()) { // Single tag result
        var estimate = poseEstimator.estimatePnpDistanceTrigSolvePose(result);
        if (estimate.isEmpty()) {
          estimate = poseEstimator.estimateLowestAmbiguityPose(result);
        }
        if (estimate.isEmpty()) {
          continue;
        }
        estimatedRobotPose = estimate.get();

        var bestTarget = result.getBestTarget();
        if (bestTarget.getFiducialId() >= 0) {
          tagIds.add((short) bestTarget.getFiducialId());
        }
        ambiguity =
            estimatedRobotPose.strategy == PhotonPoseEstimator.PoseStrategy.PNP_DISTANCE_TRIG_SOLVE
                ? 0.0
                : bestTarget.poseAmbiguity;
        tagCount = 1;
        averageTagDistance = bestTarget.bestCameraToTarget.getTranslation().getNorm();
      } else {
        continue;
      }

      poseObservations.add(
          new PoseObservation(
              estimatedRobotPose.timestampSeconds,
              estimatedRobotPose.estimatedPose,
              ambiguity,
              tagCount,
              averageTagDistance,
              PoseObservationType.PHOTONVISION));
    }

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }
}
