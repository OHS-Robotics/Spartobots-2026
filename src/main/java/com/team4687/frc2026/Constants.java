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
    public static final double deadband = 0.125;
  }

  public static final double MAX_SPEED = Units.feetToMeters(13.5); // test value: actual 13.5
  public static final double MAX_ACCELERATION = Units.feetToMeters(4.1); // auto max accel: 2.5m/s
  public static final double MAX_ROTATIONAL_SPEED = Units.degreesToRadians(270.0);
  public static final double MIN_AUTO_ROTATIONAL_SPEED = Units.degreesToRadians(60.0);
  public static final double MAX_ROTATIONAL_ACCELERATION = Units.degreesToRadians(90.0);

  public static final double AUTO_ROTATION_SPEED = Units.degreesToRadians(90.0);
  public static final double AUTO_TRANSLATION_SPEED = Units.feetToMeters(3.0);

  public static AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(
    AprilTagFields.k2026RebuiltWelded
  );

  // main robot
 public static final Transform3d[] cameraPositions = {
      new Transform3d(
          new Translation3d(
            Units.inchesToMeters(-10.25 + 1.5), Units.inchesToMeters(10.25-2), Units.inchesToMeters(8)),
          new Rotation3d(0, Units.degreesToRadians(-40.0), Units.degreesToRadians(225))),

      new Transform3d(
          new Translation3d(
            Units.inchesToMeters(-10.25 + 1.5), Units.inchesToMeters(-10.25 + 2), Units.inchesToMeters(8)),
          new Rotation3d(0, Units.degreesToRadians(-40.0), Units.degreesToRadians(180-45)))
  };
  public static final String[] cameraNames = {"Arducam_Left_Front", "Arducam_Right_Front"};

  // test robot
  /*public static final Transform3d[] cameraPositions = {
      new Transform3d(
          new Translation3d(
            Units.inchesToMeters(12.0), Units.inchesToMeters(-10.5), Units.inchesToMeters(11)),
          new Rotation3d(0, Units.degreesToRadians(45.0), Units.degreesToRadians(180)))
  };
  public static final String[] cameraNames = {"Arducam_Backup"};*/

  public static final Pose2d redHub = new Pose2d(11.9, 4.0, new Rotation2d(Math.PI));
  public static final Pose2d blueHub = new Pose2d(4.0, 4.0, new Rotation2d(0));
}