package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {
  private PhotonPoseEstimator poseEstimator;
  // todo: change the name of this camera because it's very long
  private PhotonCamera camera;
  private boolean initialized = false;

  public Matrix<N3, N1> stdDevs;

  public void initVision() {
    poseEstimator = new PhotonPoseEstimator(Constants.FIELD_LAYOUT, VisionConstants.robotToCamera0);
    camera = new PhotonCamera("Arducam_OV9281_USB_Camera");
    initialized = true;
  }

  public void updateStdDevs(
      Optional<EstimatedRobotPose> currentPose, List<PhotonTrackedTarget> targets) {
    /*
     * Get the standard deviations for pipeline results.
     * I'm still not 100% sure what this is *supposed* to be measuring,
     * but the PhotonVision documentation shows it measuring the average
     * distance between each target(apriltag) and the current estimated pose.
     * At some point I think it might be interesting to test what
     * happens if you measure the average distance between each
     * result and the center point of all of the results.
     * This might not even work, but it could be interesting.
     * Essentially copied from the PhotonLib example
     */

    double averageDistance = 0.0;
    int measuredTags = 0;

    if (currentPose.isEmpty()) {
      stdDevs = VecBuilder.fill(4.0, 4.0, 8.0);
      return;
    }

    for (var target : targets) {
      Optional<Pose3d> pose = poseEstimator.getFieldTags().getTagPose(target.getFiducialId());
      if (pose.isEmpty()) {
        continue;
      }
      measuredTags++;
      averageDistance +=
          pose.get()
              .toPose2d()
              .getTranslation()
              .getDistance(currentPose.get().estimatedPose.toPose2d().getTranslation());
    }

    if (measuredTags == 0) {
      stdDevs = VecBuilder.fill(4.0, 4.0, 8.0);
    } else {
      averageDistance /= measuredTags;

      if (measuredTags > 1) {
        stdDevs = VecBuilder.fill(0.5, 0.5, 1.0);
      } else {
        stdDevs = VecBuilder.fill(1.0, 1.0, 2.0);
      }

      if (measuredTags == 1 && averageDistance > 4.0) {
        stdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
      } else {
        stdDevs = stdDevs.times(1.0 + ((averageDistance * averageDistance) / 30.0));
      }
    }
  }

  public void updatePoseEstimate(Drive drive) {
    if (!initialized) return;

    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    Optional<EstimatedRobotPose> visionEstimatedPose = Optional.empty();

    for (var result : results) {
      visionEstimatedPose = poseEstimator.estimateCoprocMultiTagPose(result);
      if (visionEstimatedPose.isEmpty()) {
        visionEstimatedPose = poseEstimator.estimateLowestAmbiguityPose(result);
      }

      updateStdDevs(visionEstimatedPose, result.getTargets());

      visionEstimatedPose.ifPresent(
          est -> {
            drive.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, stdDevs);
          });
    }
  }
}
