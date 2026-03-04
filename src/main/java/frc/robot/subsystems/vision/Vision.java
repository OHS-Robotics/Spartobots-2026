// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
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

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
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
          "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
          tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
          robotPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/ProcessedResultCount",
          inputs[cameraIndex].processedResultCount);
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/DetectedTargetCount",
          inputs[cameraIndex].detectedTargetCount);
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/LatestResultTimestampSeconds",
          inputs[cameraIndex].latestResultTimestampSeconds);
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/AcceptedPoseCount",
          acceptedPoseCount);
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RejectedPoseCount",
          rejectedPoseCount);
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/Rejected/NoTags", rejectedNoTags);
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/Rejected/Ambiguity",
          rejectedAmbiguity);
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/Rejected/Z", rejectedZ);
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/Rejected/OutOfField",
          rejectedOutOfField);
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
    Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/ProcessedResultCount", totalProcessedResultCount);
    Logger.recordOutput("Vision/Summary/DetectedTargetCount", totalDetectedTargetCount);
    Logger.recordOutput("Vision/Summary/AcceptedPoseCount", totalAcceptedPoseCount);
    Logger.recordOutput("Vision/Summary/RejectedPoseCount", totalRejectedPoseCount);
    Logger.recordOutput("Vision/Summary/Rejected/NoTags", totalRejectedNoTags);
    Logger.recordOutput("Vision/Summary/Rejected/Ambiguity", totalRejectedAmbiguity);
    Logger.recordOutput("Vision/Summary/Rejected/Z", totalRejectedZ);
    Logger.recordOutput("Vision/Summary/Rejected/OutOfField", totalRejectedOutOfField);
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
