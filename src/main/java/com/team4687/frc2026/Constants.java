// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team4687.frc2026;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double deadband = 0.7;
  }

  public static final double MAX_SPEED = Units.feetToMeters(3.0);

  public static final class VisionConstants {
    public static final String CAMERA_NAME = "photonvision";
    public static final Transform3d ROBOT_TO_CAMERA =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(10.0), 0.0, Units.inchesToMeters(8.0)),
            new Rotation3d());
    public static final AprilTagFields FIELD = AprilTagFields.kDefaultField;
    public static final Matrix<N3, N1> VISION_MEASUREMENT_STD_DEVS =
        VecBuilder.fill(0.7, 0.7, Units.degreesToRadians(45.0));
    public static final double MAX_POSE_AMBIGUITY = 0.3;
    public static final double MAX_TRANSLATION_JUMP_METERS = 1.5;
    public static final double MAX_HEADING_ERROR_RADIANS = Units.degreesToRadians(35.0);
    public static final double STD_DEV_SCALE_SINGLE_TAG = 2.5;
  }
}
