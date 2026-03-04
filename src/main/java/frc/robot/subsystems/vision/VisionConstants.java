// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveConstants;

public class VisionConstants {
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
  public static String camera0Name = "Arducam_Left_Front";
  public static String camera1Name = "Arducam_Right_Front";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCamera0 =
      new Transform3d(
          new Translation3d(
              -DriveConstants.trackWidth / 2.0,
              DriveConstants.wheelBase / 2.0,
              Units.inchesToMeters(4.0)),
          new Rotation3d(0.0, Units.degreesToRadians(-40), Units.degreesToRadians(135)));
  public static Transform3d robotToCamera1 =
      new Transform3d(
          -DriveConstants.trackWidth / 2.0,
          -DriveConstants.wheelBase / 2.0,
          Units.inchesToMeters(4.0),
          new Rotation3d(0.0, Units.degreesToRadians(-40), Units.degreesToRadians(-135)));

  public static int camera0Pipeline = 0;
  public static int camera1Pipeline = 1;

  // Basic filtering thresholds
  // Single-tag solves above this ambiguity tend to be unstable in-match.
  public static double maxAmbiguity = 0.8;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
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
}
