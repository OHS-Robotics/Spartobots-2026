package com.team4687.frc2026.subsystems.vision.fusion;

import com.team4687.frc2026.subsystems.vision.estimation.VisionPoseEstimate;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * Applies sanity checks and weighting to vision estimates before fusing them into drivetrain
 * odometry.
 */
public class VisionSensorFusion {
  public record VisionMeasurement(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs) {}

  private final Supplier<Rotation2d> headingSupplier;
  private final Matrix<N3, N1> baseStdDevs;
  private final VisionFusionConfig config;

  public VisionSensorFusion(
      Supplier<Rotation2d> headingSupplier, Matrix<N3, N1> baseStdDevs, VisionFusionConfig config) {
    this.headingSupplier = headingSupplier;
    this.baseStdDevs = baseStdDevs;
    this.config = config;
  }

  public Optional<VisionMeasurement> fuse(VisionPoseEstimate estimate, Pose2d odometryPose) {
    if (!isTrusted(estimate, odometryPose)) {
      return Optional.empty();
    }

    return Optional.of(
        new VisionMeasurement(
            estimate.pose(), estimate.timestampSeconds(), selectStdDevs(estimate)));
  }

  private boolean isTrusted(VisionPoseEstimate estimate, Pose2d odometryPose) {
    if (estimate.tagCount() <= 0) {
      return false;
    }
    if (estimate.maxAmbiguity() > config.maxPoseAmbiguity()) {
      return false;
    }

    double translationDelta =
        estimate.pose().getTranslation().getDistance(odometryPose.getTranslation());
    if (translationDelta > config.maxTranslationJumpMeters()) {
      return false;
    }

    double headingErrorRadians =
        Math.abs(estimate.pose().getRotation().minus(headingSupplier.get()).getRadians());
    return headingErrorRadians <= config.maxHeadingErrorRadians();
  }

  private Matrix<N3, N1> selectStdDevs(VisionPoseEstimate estimate) {
    if (estimate.tagCount() > 1) {
      return baseStdDevs;
    }

    return VecBuilder.fill(
        baseStdDevs.get(0, 0) * config.stdDevScaleSingleTag(),
        baseStdDevs.get(1, 1) * config.stdDevScaleSingleTag(),
        baseStdDevs.get(2, 2) * config.stdDevScaleSingleTag());
  }
}

