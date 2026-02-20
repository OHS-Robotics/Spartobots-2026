// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team4687.frc2026;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
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
    public static final int kManipulatorControllerPort = 1;
    public static final double deadband = 0.05;
  }

  public static final double MAX_SPEED = Units.feetToMeters(13.5);
  public static final double MAX_ACCELERATION = Units.feetToMeters(6.7);
  public static final double MAX_ROTATIONAL_SPEED = Units.degreesToRadians(270.0);
  public static final double MIN_AUTO_ROTATIONAL_SPEED = Units.degreesToRadians(120.0);
  public static final double MAX_ROTATIONAL_ACCELERATION = Units.degreesToRadians(45.0);
  public static AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(
    AprilTagFields.k2026RebuiltWelded
  );

  public static final Transform3d cameraPosition = new Transform3d(new Translation3d(
    Units.inchesToMeters(0),
    Units.inchesToMeters(-5),
    Units.inchesToMeters(10.5)),
    new Rotation3d(0, 0, 0));

  public static final Pose2d redHub = new Pose2d(11.9, 4.0, new Rotation2d(Math.PI));
  public static final Pose2d blueHub = new Pose2d(3, 4.0, new Rotation2d(0));
}
