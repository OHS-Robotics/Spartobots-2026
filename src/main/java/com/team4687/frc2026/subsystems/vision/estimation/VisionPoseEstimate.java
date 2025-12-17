package com.team4687.frc2026.subsystems.vision.estimation;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * A minimal, implementation-agnostic pose estimate produced by a vision pose estimator.
 *
 * @param pose Estimated robot pose in field coordinates.
 * @param timestampSeconds Timestamp when the image was captured (in seconds).
 * @param tagCount Number of AprilTags/targets used to produce the estimate.
 * @param maxAmbiguity Maximum per-target pose ambiguity (0 = best, 1 = worst). If unknown, 0.
 */
public record VisionPoseEstimate(Pose2d pose, double timestampSeconds, int tagCount, double maxAmbiguity) {}
