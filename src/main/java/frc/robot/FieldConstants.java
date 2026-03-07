package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class FieldConstants {
  public static final AprilTagFieldLayout APRIL_TAG_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
  public static final double FIELD_LENGTH_METERS = APRIL_TAG_LAYOUT.getFieldLength();
  public static final double FIELD_WIDTH_METERS = APRIL_TAG_LAYOUT.getFieldWidth();
  private static final Rotation2d HALF_TURN = Rotation2d.fromRadians(Math.PI);

  // All target poses are defined from the blue alliance perspective and mirrored for red.
  public static enum FieldTarget {
    HUB_CENTER(new Pose3d(new Translation3d(4.5974, 4.034536, 1.5748), new Rotation3d())),
    OUTPOST(new Pose3d(new Translation3d(0.0, 0.665988, 0.0), new Rotation3d(0.0, 0.0, Math.PI)));

    private final Pose3d bluePose;

    private FieldTarget(Pose3d bluePose) {
      this.bluePose = bluePose;
    }

    public Pose3d bluePose() {
      return bluePose;
    }
  }

  private FieldConstants() {}

  public static Pose2d getTargetPose(FieldTarget target) {
    return getTargetPose(target, DriverStation.getAlliance().orElse(Alliance.Blue));
  }

  public static Pose3d getTargetPose3d(FieldTarget target) {
    return getTargetPose3d(target, DriverStation.getAlliance().orElse(Alliance.Blue));
  }

  public static Pose2d getTargetPose(FieldTarget target, Alliance alliance) {
    return toAlliancePose(target.bluePose().toPose2d(), alliance);
  }

  public static Pose3d getTargetPose3d(FieldTarget target, Alliance alliance) {
    return toAlliancePose(target.bluePose(), alliance);
  }

  public static Pose2d toAlliancePose(Pose2d bluePose) {
    return toAlliancePose(bluePose, DriverStation.getAlliance().orElse(Alliance.Blue));
  }

  public static Pose2d toAlliancePose(Pose2d bluePose, Alliance alliance) {
    return alliance == Alliance.Red ? flipPose(bluePose) : bluePose;
  }

  public static Pose3d toAlliancePose(Pose3d bluePose) {
    return toAlliancePose(bluePose, DriverStation.getAlliance().orElse(Alliance.Blue));
  }

  public static Pose3d toAlliancePose(Pose3d bluePose, Alliance alliance) {
    return alliance == Alliance.Red ? flipPose(bluePose) : bluePose;
  }

  public static Translation2d toAllianceTranslation(Translation2d blueTranslation) {
    return toAllianceTranslation(
        blueTranslation, DriverStation.getAlliance().orElse(Alliance.Blue));
  }

  public static Translation2d toAllianceTranslation(
      Translation2d blueTranslation, Alliance alliance) {
    return alliance == Alliance.Red ? flipTranslation(blueTranslation) : blueTranslation;
  }

  private static Pose2d flipPose(Pose2d bluePose) {
    return new Pose2d(
        flipTranslation(bluePose.getTranslation()), bluePose.getRotation().plus(HALF_TURN));
  }

  private static Pose3d flipPose(Pose3d bluePose) {
    Pose2d redPose2d = flipPose(bluePose.toPose2d());
    return new Pose3d(
        redPose2d.getX(),
        redPose2d.getY(),
        bluePose.getZ(),
        new Rotation3d(0.0, 0.0, redPose2d.getRotation().getRadians()));
  }

  private static Translation2d flipTranslation(Translation2d blueTranslation) {
    return new Translation2d(
        FIELD_LENGTH_METERS - blueTranslation.getX(), FIELD_WIDTH_METERS - blueTranslation.getY());
  }
}
