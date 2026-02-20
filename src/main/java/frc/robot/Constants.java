// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static AprilTagFieldLayout FIELD_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  public static double halfOfFieldX = Units.inchesToMeters(561.11 / 2);

  public static Transform3d cameraPosition =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(0), Units.inchesToMeters(-5), Units.inchesToMeters(10.5)),
          new Rotation3d(0, 0, 0));

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final double robotBaseLength = 23.5;
  // auto align poses
  public static final Pose2d redHub =
      new Pose2d(
          Units.inchesToMeters(469.111), Units.inchesToMeters(158.844), new Rotation2d(Math.PI));
  public static final Pose2d blueHub =
      new Pose2d(Units.inchesToMeters(182.111), Units.inchesToMeters(158.844), new Rotation2d(0));

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final double fieldLength = Units.inchesToMeters(651.22);
  public static final double fieldWidth = Units.inchesToMeters(317.69);

  // break the field into 8 sections to determine which trench to drive through
  public static final double blueLine = Units.inchesToMeters(182.11);
  public static final double midLineX = fieldLength / 2;
  public static final double redLine = fieldLength - Units.inchesToMeters(182.11);
  public static final double midLineY = fieldWidth / 2;

  // define trench entrance positions
  public static final double blueInnerX = 3.225;
  public static final double blueOuterX = 5.9;
  public static final double redOuterX = 10.6;
  public static final double redInnerX = 13.3;
  public static final double topY = 7.42;
  public static final double bottomY = 0.6;
  public static final Translation2d blueTrenchTopInner = new Translation2d(blueInnerX, topY);
  public static final Translation2d blueTrenchTopOuter = new Translation2d(blueOuterX, topY);
  public static final Translation2d redTrenchTopOuter = new Translation2d(redOuterX, topY);
  public static final Translation2d redTrenchTopInner = new Translation2d(redInnerX, topY);
  public static final Translation2d blueTrenchBottomInner = new Translation2d(blueInnerX, bottomY);
  public static final Translation2d blueTrenchBottomOuter = new Translation2d(blueOuterX, bottomY);
  public static final Translation2d redTrenchBottomOuter = new Translation2d(redOuterX, bottomY);
  public static final Translation2d redTrenchBottomInner = new Translation2d(redInnerX, bottomY);

  // define outpost positions
  public static final Pose2d blueOutpost = new Pose2d(.5, .66, new Rotation2d(0));
  public static final Pose2d blueOutpostBefore = new Pose2d(1, .66, new Rotation2d(0));
  public static final Pose2d redOutpost =
      new Pose2d(16, 7.38, new Rotation2d(Units.degreesToRadians(180)));
  public static final Pose2d redOutpostBefore =
      new Pose2d(15.5, 7.38, new Rotation2d(Units.degreesToRadians(180)));
}
