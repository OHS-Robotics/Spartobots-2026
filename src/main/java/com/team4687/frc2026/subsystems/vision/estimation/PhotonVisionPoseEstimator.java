package com.team4687.frc2026.subsystems.vision.estimation;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * PhotonVision-backed implementation of {@link VisionPoseEstimator}.
 *
 * <p>This class owns the PhotonVision camera handle and pose estimator configuration.
 */
public class PhotonVisionPoseEstimator implements VisionPoseEstimator {
  private final PhotonCamera camera;
  private final PhotonPoseEstimator estimator;
  private Alliance lastAlliance;

  public PhotonVisionPoseEstimator(
      String cameraName, Transform3d robotToCamera, AprilTagFields field) {
    camera = new PhotonCamera(cameraName);

    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(field);
    fieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

    estimator =
        new PhotonPoseEstimator(
            fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
    estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  @Override
  public Optional<VisionPoseEstimate> estimate(Pose2d referencePose) {
    updateFieldOrigin();
    estimator.setReferencePose(referencePose);

    return estimator
        .update(camera.getLatestResult())
        .map(
            estimatedRobotPose -> {
              int tagCount = estimatedRobotPose.targetsUsed.size();
              double maxAmbiguity =
                  estimatedRobotPose.targetsUsed.stream()
                      .mapToDouble(PhotonTrackedTarget::getPoseAmbiguity)
                      .map(ambiguity -> ambiguity < 0.0 ? 0.0 : ambiguity)
                      .max()
                      .orElse(0.0);

              return new VisionPoseEstimate(
                  estimatedRobotPose.estimatedPose.toPose2d(),
                  estimatedRobotPose.timestampSeconds,
                  tagCount,
                  maxAmbiguity);
            });
  }

  private void updateFieldOrigin() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty() || alliance.get() == lastAlliance) {
      return;
    }

    lastAlliance = alliance.get();
    estimator
        .getFieldTags()
        .setOrigin(
            lastAlliance == Alliance.Red
                ? OriginPosition.kRedAllianceWallRightSide
                : OriginPosition.kBlueAllianceWallRightSide);
  }
}

