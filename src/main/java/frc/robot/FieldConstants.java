package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class FieldConstants {
  public static final AprilTagFieldLayout APRIL_TAG_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
  public static final double FIELD_LENGTH_METERS = APRIL_TAG_LAYOUT.getFieldLength();
  public static final double FIELD_WIDTH_METERS = APRIL_TAG_LAYOUT.getFieldWidth();
  private static final Rotation2d HALF_TURN = Rotation2d.fromRadians(Math.PI);

  private FieldConstants() {}

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
