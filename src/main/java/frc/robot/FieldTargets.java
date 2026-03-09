package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class FieldTargets {
  private static final Rotation2d HALF_TURN = Rotation2d.fromRadians(Math.PI);
  private static final double FUEL_GRID_X_SPACING_METERS = Units.inchesToMeters(5.991);
  private static final double FUEL_GRID_Y_SPACING_METERS = Units.inchesToMeters(5.95);
  private static final double ALLIANCE_ZONE_DEPTH_METERS = Units.inchesToMeters(158.6);
  private static final double AUTO_START_X_METERS =
      ALLIANCE_ZONE_DEPTH_METERS - Units.inchesToMeters(24.0);
  private static final double PARK_ZONE_MIN_X_METERS = Units.inchesToMeters(30.0);
  private static final double PARK_ZONE_MAX_X_METERS = Units.inchesToMeters(120.0);
  private static final double LOWER_PARK_ZONE_MIN_Y_METERS = Units.inchesToMeters(36.0);
  private static final double LOWER_PARK_ZONE_MAX_Y_METERS = Units.inchesToMeters(96.0);

  public static final AlliancePoseTarget3d HUB =
      mirroredPose3d("Hub", new Pose3d(4.5974, 4.034536, 1.5748, new Rotation3d()));

  public static final AlliancePoseTarget2d OUTPOST =
      explicitPose2d(
          "Outpost",
          new Pose2d(0.0, 0.665988, HALF_TURN),
          new Pose2d(16.621, 7.403338, Rotation2d.kZero));

  public static final AllianceZoneTarget DEPOT_INTAKE =
      explicitZone(
          "DepotIntake",
          gridZone("BlueDepotIntake", new Translation2d(16.0274, 1.646936), 4, 6),
          gridZone("RedDepotIntake", new Translation2d(0.02, 5.53), 4, 6));

  public static final FieldZone NEUTRAL_FLOOR_INTAKE =
      gridZone("NeutralFloorIntake", new Translation2d(7.35737, 1.724406), 12, 30);

  // REBUILT has tower contact/level endgame scoring, not an official "park zone". These are
  // team-defined staging rectangles inside the alliance zone for approach/parking behavior.
  public static final AllianceZoneTarget LOWER_PARK_ZONE =
      mirroredZone(
          "LowerParkZone",
          new FieldZone(
              "BlueLowerParkZone",
              new Translation2d(PARK_ZONE_MIN_X_METERS, LOWER_PARK_ZONE_MIN_Y_METERS),
              new Translation2d(PARK_ZONE_MAX_X_METERS, LOWER_PARK_ZONE_MAX_Y_METERS)));

  public static final AllianceZoneTarget UPPER_PARK_ZONE =
      mirroredZone(
          "UpperParkZone",
          new FieldZone(
              "BlueUpperParkZone",
              new Translation2d(
                  PARK_ZONE_MIN_X_METERS,
                  FieldConstants.FIELD_WIDTH_METERS - LOWER_PARK_ZONE_MAX_Y_METERS),
              new Translation2d(
                  PARK_ZONE_MAX_X_METERS,
                  FieldConstants.FIELD_WIDTH_METERS - LOWER_PARK_ZONE_MIN_Y_METERS)));

  public static final AlliancePoseTarget2d AUTO_START_LOWER =
      mirroredPose2d(
          "AutoStartLower",
          new Pose2d(AUTO_START_X_METERS, LOWER_PARK_ZONE_MAX_Y_METERS, Rotation2d.kZero));

  public static final AlliancePoseTarget2d AUTO_START_CENTER =
      mirroredPose2d(
          "AutoStartCenter",
          new Pose2d(
              AUTO_START_X_METERS, FieldConstants.FIELD_WIDTH_METERS / 2.0, Rotation2d.kZero));

  public static final AlliancePoseTarget2d AUTO_START_UPPER =
      mirroredPose2d(
          "AutoStartUpper",
          new Pose2d(
              AUTO_START_X_METERS,
              FieldConstants.FIELD_WIDTH_METERS - LOWER_PARK_ZONE_MAX_Y_METERS,
              Rotation2d.kZero));

  private FieldTargets() {}

  private static AlliancePoseTarget2d mirroredPose2d(String name, Pose2d bluePose) {
    return explicitPose2d(name, bluePose, FieldConstants.toAlliancePose(bluePose, Alliance.Red));
  }

  private static AlliancePoseTarget2d explicitPose2d(String name, Pose2d bluePose, Pose2d redPose) {
    return new AlliancePoseTarget2d(name, bluePose, redPose);
  }

  private static AlliancePoseTarget3d mirroredPose3d(String name, Pose3d bluePose) {
    return explicitPose3d(name, bluePose, FieldConstants.toAlliancePose(bluePose, Alliance.Red));
  }

  private static AlliancePoseTarget3d explicitPose3d(String name, Pose3d bluePose, Pose3d redPose) {
    return new AlliancePoseTarget3d(name, bluePose, redPose);
  }

  private static AllianceZoneTarget mirroredZone(String name, FieldZone blueZone) {
    return explicitZone(name, blueZone, mirrorZone(name + "Red", blueZone));
  }

  private static AllianceZoneTarget explicitZone(
      String name, FieldZone blueZone, FieldZone redZone) {
    return new AllianceZoneTarget(name, blueZone, redZone);
  }

  private static FieldZone gridZone(
      String name, Translation2d minimumGridPoint, int columns, int rows) {
    double halfSpacingX = FUEL_GRID_X_SPACING_METERS / 2.0;
    double halfSpacingY = FUEL_GRID_Y_SPACING_METERS / 2.0;
    double maxX = minimumGridPoint.getX() + ((columns - 1) * FUEL_GRID_X_SPACING_METERS);
    double maxY = minimumGridPoint.getY() + ((rows - 1) * FUEL_GRID_Y_SPACING_METERS);

    return new FieldZone(
        name,
        new Translation2d(
            minimumGridPoint.getX() - halfSpacingX, minimumGridPoint.getY() - halfSpacingY),
        new Translation2d(maxX + halfSpacingX, maxY + halfSpacingY));
  }

  private static FieldZone mirrorZone(String name, FieldZone blueZone) {
    return new FieldZone(
        name,
        FieldConstants.toAllianceTranslation(blueZone.minCorner(), Alliance.Red),
        FieldConstants.toAllianceTranslation(blueZone.maxCorner(), Alliance.Red));
  }

  public record FieldZone(String name, Translation2d minCorner, Translation2d maxCorner)
      implements FieldTarget {
    public FieldZone {
      double normalizedMinX = Math.min(minCorner.getX(), maxCorner.getX());
      double normalizedMinY = Math.min(minCorner.getY(), maxCorner.getY());
      double normalizedMaxX = Math.max(minCorner.getX(), maxCorner.getX());
      double normalizedMaxY = Math.max(minCorner.getY(), maxCorner.getY());
      minCorner = new Translation2d(normalizedMinX, normalizedMinY);
      maxCorner = new Translation2d(normalizedMaxX, normalizedMaxY);
    }

    public Translation2d center() {
      return new Translation2d(
          (minCorner.getX() + maxCorner.getX()) / 2.0, (minCorner.getY() + maxCorner.getY()) / 2.0);
    }

    public Pose2d centerPose(Rotation2d rotation) {
      return new Pose2d(center(), rotation);
    }

    public boolean contains(Translation2d point) {
      return point.getX() >= minCorner.getX()
          && point.getX() <= maxCorner.getX()
          && point.getY() >= minCorner.getY()
          && point.getY() <= maxCorner.getY();
    }
  }

  public record AlliancePoseTarget2d(String name, Pose2d bluePose, Pose2d redPose)
      implements FieldTarget {
    public Pose2d forAlliance(Alliance alliance) {
      return alliance == Alliance.Red ? redPose : bluePose;
    }

    public Pose2d forOpponent(Alliance alliance) {
      return alliance == Alliance.Red ? bluePose : redPose;
    }
  }

  public record AlliancePoseTarget3d(String name, Pose3d bluePose, Pose3d redPose)
      implements FieldTarget {
    public Pose3d forAlliance(Alliance alliance) {
      return alliance == Alliance.Red ? redPose : bluePose;
    }

    public Pose3d forOpponent(Alliance alliance) {
      return alliance == Alliance.Red ? bluePose : redPose;
    }
  }

  public record AllianceZoneTarget(String name, FieldZone blueZone, FieldZone redZone)
      implements FieldTarget {
    public FieldZone forAlliance(Alliance alliance) {
      return alliance == Alliance.Red ? redZone : blueZone;
    }

    public FieldZone forOpponent(Alliance alliance) {
      return alliance == Alliance.Red ? blueZone : redZone;
    }
  }
}
