// Copyright (c) 2021-2026 Littleton Robotics
// Copyright (c) 2026 Team 4687 Spartobots
//
// SPDX-License-Identifier: BSD-3-Clause

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionConstants.CameraVisualizationConfig;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.util.NetworkTablesUtil;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final Supplier<Pose2d> robotPoseSupplier;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private VisionPoseEstimate bestRobotPoseEstimate = null;

  public Vision(VisionConsumer consumer, Supplier<Pose2d> robotPoseSupplier, VisionIO... io) {
    this.consumer = consumer;
    this.robotPoseSupplier = robotPoseSupplier;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  /** Returns the best accepted robot pose observation from the most recent cycle. */
  public Optional<VisionPoseEstimate> getBestRobotPoseEstimate() {
    return Optional.ofNullable(bestRobotPoseEstimate);
  }

  @Override
  public void periodic() {
    bestRobotPoseEstimate = null;

    for (var cameraIo : io) {
      if (cameraIo instanceof VisionIOPhotonVisionSim simIo) {
        simIo.updateSimulationPose();
        break;
      }
    }

    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs(
          NetworkTablesUtil.logPath("Vision/Camera" + Integer.toString(i)), inputs[i]);
      logCameraVisualization(cameraIndexToLogKey(i), i);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();
    int totalProcessedResultCount = 0;
    int totalDetectedTargetCount = 0;
    int totalAcceptedPoseCount = 0;
    int totalRejectedPoseCount = 0;
    int totalRejectedNoTags = 0;
    int totalRejectedAmbiguity = 0;
    int totalRejectedZ = 0;
    int totalRejectedOutOfField = 0;

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);
      totalProcessedResultCount += inputs[cameraIndex].processedResultCount;
      totalDetectedTargetCount += inputs[cameraIndex].detectedTargetCount;

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();
      int acceptedPoseCount = 0;
      int rejectedPoseCount = 0;
      int rejectedNoTags = 0;
      int rejectedAmbiguity = 0;
      int rejectedZ = 0;
      int rejectedOutOfField = 0;

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        boolean noTags = observation.tagCount() == 0;
        boolean highAmbiguity =
            observation.tagCount() == 1
                && observation.ambiguity() > maxAmbiguity; // Cannot be high ambiguity
        boolean badZ = Math.abs(observation.pose().getZ()) > maxZError; // Must be realistic Z
        boolean outOfField =
            // Must be within the field boundaries
            observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth();

        // Check whether to reject pose
        boolean rejectPose = noTags || highAmbiguity || badZ || outOfField;

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          rejectedPoseCount++;
          if (noTags) {
            rejectedNoTags++;
          }
          if (highAmbiguity) {
            rejectedAmbiguity++;
          }
          if (badZ) {
            rejectedZ++;
          }
          if (outOfField) {
            rejectedOutOfField++;
          }
          robotPosesRejected.add(observation.pose());
        } else {
          acceptedPoseCount++;
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        VisionPoseEstimate candidateEstimate =
            new VisionPoseEstimate(
                observation.pose().toPose2d(),
                observation.timestamp(),
                observation.tagCount(),
                observation.averageTagDistance(),
                observation.ambiguity(),
                cameraIndex);
        if (isBetterRobotPoseEstimate(candidateEstimate, bestRobotPoseEstimate)) {
          bestRobotPoseEstimate = candidateEstimate;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera metadata
      Logger.recordOutput(
          cameraIndexToLogKey(cameraIndex) + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          cameraIndexToLogKey(cameraIndex) + "/RobotPoses", robotPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          cameraIndexToLogKey(cameraIndex) + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[0]));
      Logger.recordOutput(
          cameraIndexToLogKey(cameraIndex) + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[0]));
      Logger.recordOutput(
          cameraIndexToLogKey(cameraIndex) + "/ProcessedResultCount",
          inputs[cameraIndex].processedResultCount);
      Logger.recordOutput(
          cameraIndexToLogKey(cameraIndex) + "/DetectedTargetCount",
          inputs[cameraIndex].detectedTargetCount);
      Logger.recordOutput(
          cameraIndexToLogKey(cameraIndex) + "/LatestResultTimestampSeconds",
          inputs[cameraIndex].latestResultTimestampSeconds);
      Logger.recordOutput(
          cameraIndexToLogKey(cameraIndex) + "/AcceptedPoseCount", acceptedPoseCount);
      Logger.recordOutput(
          cameraIndexToLogKey(cameraIndex) + "/RejectedPoseCount", rejectedPoseCount);
      Logger.recordOutput(cameraIndexToLogKey(cameraIndex) + "/Rejected/NoTags", rejectedNoTags);
      Logger.recordOutput(
          cameraIndexToLogKey(cameraIndex) + "/Rejected/Ambiguity", rejectedAmbiguity);
      Logger.recordOutput(cameraIndexToLogKey(cameraIndex) + "/Rejected/Z", rejectedZ);
      Logger.recordOutput(
          cameraIndexToLogKey(cameraIndex) + "/Rejected/OutOfField", rejectedOutOfField);
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
      totalAcceptedPoseCount += acceptedPoseCount;
      totalRejectedPoseCount += rejectedPoseCount;
      totalRejectedNoTags += rejectedNoTags;
      totalRejectedAmbiguity += rejectedAmbiguity;
      totalRejectedZ += rejectedZ;
      totalRejectedOutOfField += rejectedOutOfField;
    }

    // Log summary data
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Vision/Summary/TagPoses"), allTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Vision/Summary/RobotPoses"),
        allRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Vision/Summary/RobotPosesAccepted"),
        allRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Vision/Summary/RobotPosesRejected"),
        allRobotPosesRejected.toArray(new Pose3d[0]));
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Vision/Summary/ProcessedResultCount"),
        totalProcessedResultCount);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Vision/Summary/DetectedTargetCount"), totalDetectedTargetCount);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Vision/Summary/AcceptedPoseCount"), totalAcceptedPoseCount);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Vision/Summary/RejectedPoseCount"), totalRejectedPoseCount);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Vision/Summary/Rejected/NoTags"), totalRejectedNoTags);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Vision/Summary/Rejected/Ambiguity"), totalRejectedAmbiguity);
    Logger.recordOutput(NetworkTablesUtil.logPath("Vision/Summary/Rejected/Z"), totalRejectedZ);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Vision/Summary/Rejected/OutOfField"), totalRejectedOutOfField);
  }

  private void logCameraVisualization(String cameraLogKey, int cameraIndex) {
    Transform3d robotToCamera = getRobotToCamera(cameraIndex);
    CameraVisualizationConfig visualizationConfig = getCameraVisualizationConfig(cameraIndex);
    if (robotToCamera == null || visualizationConfig == null) {
      Logger.recordOutput(cameraLogKey + "/CameraPose", new Pose3d());
      Logger.recordOutput(cameraLogKey + "/Frustum", new Pose3d[] {});
      return;
    }

    Pose3d cameraPose = new Pose3d(robotPoseSupplier.get()).plus(robotToCamera);
    Logger.recordOutput(cameraLogKey + "/CameraPose", cameraPose);
    Logger.recordOutput(
        cameraLogKey + "/Frustum", buildFrustumGeometry(cameraPose, visualizationConfig));
  }

  private static String cameraIndexToLogKey(int cameraIndex) {
    return NetworkTablesUtil.logPath("Vision/Camera" + Integer.toString(cameraIndex));
  }

  private static Pose3d[] buildFrustumGeometry(
      Pose3d cameraPose, CameraVisualizationConfig visualizationConfig) {
    double frustumLengthMeters = visualizationConfig.frustumLengthMeters();
    double halfVerticalFovTangent = getHalfVerticalFovTangent(visualizationConfig);
    double halfHorizontalFovTangent =
        halfVerticalFovTangent
            * ((double) visualizationConfig.width() / (double) visualizationConfig.height());

    double halfWidthMeters = frustumLengthMeters * halfHorizontalFovTangent;
    double halfHeightMeters = frustumLengthMeters * halfVerticalFovTangent;

    Pose3d topLeft =
        transformCameraRelative(cameraPose, frustumLengthMeters, halfWidthMeters, halfHeightMeters);
    Pose3d topRight =
        transformCameraRelative(
            cameraPose, frustumLengthMeters, -halfWidthMeters, halfHeightMeters);
    Pose3d bottomRight =
        transformCameraRelative(
            cameraPose, frustumLengthMeters, -halfWidthMeters, -halfHeightMeters);
    Pose3d bottomLeft =
        transformCameraRelative(
            cameraPose, frustumLengthMeters, halfWidthMeters, -halfHeightMeters);

    return new Pose3d[] {
      cameraPose,
      topLeft,
      topRight,
      cameraPose,
      bottomRight,
      bottomLeft,
      cameraPose,
      topLeft,
      bottomLeft,
      bottomRight,
      topRight
    };
  }

  private static Pose3d transformCameraRelative(
      Pose3d cameraPose, double xMeters, double yMeters, double zMeters) {
    return cameraPose.plus(
        new Transform3d(new Translation3d(xMeters, yMeters, zMeters), new Rotation3d()));
  }

  private static double getHalfVerticalFovTangent(CameraVisualizationConfig visualizationConfig) {
    double aspectRatio =
        (double) visualizationConfig.width() / (double) visualizationConfig.height();
    double halfDiagonalFovTangent = Math.tan(visualizationConfig.diagonalFov().getRadians() * 0.5);
    return halfDiagonalFovTangent / Math.hypot(aspectRatio, 1.0);
  }

  private static boolean isBetterRobotPoseEstimate(
      VisionPoseEstimate candidate, VisionPoseEstimate currentBest) {
    if (currentBest == null) {
      return true;
    }
    if (candidate.tagCount() != currentBest.tagCount()) {
      return candidate.tagCount() > currentBest.tagCount();
    }
    if (Math.abs(candidate.averageTagDistanceMeters() - currentBest.averageTagDistanceMeters())
        > 1e-9) {
      return candidate.averageTagDistanceMeters() < currentBest.averageTagDistanceMeters();
    }
    if (Math.abs(candidate.ambiguity() - currentBest.ambiguity()) > 1e-9) {
      return candidate.ambiguity() < currentBest.ambiguity();
    }
    return candidate.timestampSeconds() > currentBest.timestampSeconds();
  }

  public static record VisionPoseEstimate(
      Pose2d pose,
      double timestampSeconds,
      int tagCount,
      double averageTagDistanceMeters,
      double ambiguity,
      int cameraIndex) {}

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
