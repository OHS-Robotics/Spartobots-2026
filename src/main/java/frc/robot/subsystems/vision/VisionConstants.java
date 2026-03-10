// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.RobotSettings;

/** Vision constants delegated from {@link frc.robot.RobotSettings}. */
public class VisionConstants {
  // Camera names, must match names configured on coprocessor
  public static final String camera0Name = RobotSettings.Vision.camera0Name;
  public static final String camera1Name = RobotSettings.Vision.camera1Name;

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static final Transform3d robotToCamera0 = RobotSettings.Vision.robotToCamera0;
  public static final Transform3d robotToCamera1 = RobotSettings.Vision.robotToCamera1;

  // Basic filtering thresholds
  public static final double maxAmbiguity = RobotSettings.Vision.maxAmbiguity;
  public static final double maxZError = RobotSettings.Vision.maxZError;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static final double linearStdDevBaseline = RobotSettings.Vision.linearStdDevBaseline;
  public static final double angularStdDevBaseline = RobotSettings.Vision.angularStdDevBaseline;

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static final double[] cameraStdDevFactors = RobotSettings.Vision.cameraStdDevFactors;

  // Multipliers to apply for MegaTag 2 observations
  public static final double linearStdDevMegatag2Factor =
      RobotSettings.Vision.linearStdDevMegatag2Factor; // More stable than full 3D solve
  public static final double angularStdDevMegatag2Factor =
      RobotSettings.Vision.angularStdDevMegatag2Factor; // No rotation data available
}
