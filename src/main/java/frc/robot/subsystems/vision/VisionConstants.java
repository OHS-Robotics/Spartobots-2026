// Copyright (c) 2021-2026 Littleton Robotics
// Copyright (c) 2026 Team 4687 Spartobots
//
// SPDX-License-Identifier: BSD-3-Clause

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class VisionConstants {
  public static record CameraVisualizationConfig(
      int width, int height, Rotation2d diagonalFov, double frustumLengthMeters) {}

  public static record CameraSimConfig(
      int width,
      int height,
      Rotation2d diagonalFov,
      double fps,
      double exposureMs,
      double averageLatencyMs,
      double latencyStdDevMs,
      double calibrationAverageErrorPx,
      double calibrationErrorStdDevPx,
      double maxSightRangeMeters) {}

  // AprilTag layout
  public static final AprilTagFieldLayout aprilTagLayout = Constants.FIELD_LAYOUT;

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "Arducam_Right";
  public static String camera1Name = "Arducam_Left";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  private static final double cameraForwardMeters = 40.3 / 1000.0;
  private static final double cameraSideOffsetMeters = 322.627 / 1000.0;
  private static final double cameraHeightMeters = 159.395 / 1000.0;
  // In WPILib's NWU frame, a camera tilted upward uses a negative pitch rotation.
  private static final double cameraPitchDegrees = -22.5;

  public static Transform3d robotToCamera0 =
      new Transform3d(
          new Translation3d(cameraForwardMeters, -cameraSideOffsetMeters, cameraHeightMeters),
          new Rotation3d(
              0.0, Units.degreesToRadians(cameraPitchDegrees), Units.degreesToRadians(-90.0)));
  public static Transform3d robotToCamera1 =
      new Transform3d(
          new Translation3d(cameraForwardMeters, cameraSideOffsetMeters, cameraHeightMeters),
          new Rotation3d(
              0.0, Units.degreesToRadians(cameraPitchDegrees), Units.degreesToRadians(90.0)));

  public static int camera0Pipeline = 0;
  public static int camera1Pipeline = 1;

  // Basic filtering thresholds
  // Single-tag solves above this ambiguity tend to be unstable in-match.
  public static double maxAmbiguity = 0.8;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 2.5; // Meters
  public static double angularStdDevBaseline = 2.5; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        2.0, // Camera 0
        2.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available

  public static final CameraSimConfig camera0SimConfig =
      new CameraSimConfig(
          1280, 800, Rotation2d.fromDegrees(79.0), 35.0, 12.0, 30.0, 5.0, 0.25, 0.08, 7.0);
  public static final CameraSimConfig camera1SimConfig =
      new CameraSimConfig(
          960, 720, Rotation2d.fromDegrees(90.0), 25.0, 16.0, 40.0, 7.0, 0.35, 0.12, 6.0);

  public static final CameraVisualizationConfig camera0VisualizationConfig =
      new CameraVisualizationConfig(
          camera0SimConfig.width(),
          camera0SimConfig.height(),
          camera0SimConfig.diagonalFov(),
          camera0SimConfig.maxSightRangeMeters());
  public static final CameraVisualizationConfig camera1VisualizationConfig =
      new CameraVisualizationConfig(
          camera1SimConfig.width(),
          camera1SimConfig.height(),
          camera1SimConfig.diagonalFov(),
          camera1SimConfig.maxSightRangeMeters());

  public static Transform3d getRobotToCamera(int cameraIndex) {
    return switch (cameraIndex) {
      case 0 -> robotToCamera0;
      case 1 -> robotToCamera1;
      default -> null;
    };
  }

  public static CameraVisualizationConfig getCameraVisualizationConfig(int cameraIndex) {
    return switch (cameraIndex) {
      case 0 -> camera0VisualizationConfig;
      case 1 -> camera1VisualizationConfig;
      default -> null;
    };
  }
}
