package com.team4687.frc2026.subsystems.vision.estimation;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.Optional;

/**
 * Pluggable module that returns a field-relative robot pose estimate from vision.
 *
 * <p>Keeping this interface small makes it easy to swap in a different vision implementation later
 * (PhotonVision, Limelight, custom coprocessor, simulation, etc.).
 */
public interface VisionPoseEstimator {
  Optional<VisionPoseEstimate> estimate(Pose2d referencePose);
}

