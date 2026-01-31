// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team4687.frc2026;

import java.io.File;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;

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
  public static final double MAX_ROTATIONAL_SPEED = Units.degreesToRadians(5.0);
  public static AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(
    AprilTagFields.k2026RebuiltWelded
  );

  public static Transform3d cameraPosition = new Transform3d(new Translation3d(.381, 0.0, .4572), new Rotation3d(0, 0, 0));

}
