package frc.robot.targeting;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;
import java.util.function.DoubleSupplier;

public class FieldTargetingService {
  private static final double trenchSafetyRetractDistanceMeters = Units.inchesToMeters(24.0);
  private static final Pose2d blueOutpostOpeningShotPose =
      new Pose2d(2.85, 2.1, Rotation2d.fromDegrees(-130.0));
  private static final Pose2d redOutpostOpeningShotPose =
      new Pose2d(
          Constants.fieldLength - 2.85, Constants.fieldWidth - 2.1, Rotation2d.fromDegrees(50.0));
  private static final Pose2d blueOutpostEndingShotPose =
      new Pose2d(2.85, 6.3, Rotation2d.fromDegrees(-50));
  private static final Pose2d redOutpostEndingShotPose =
      new Pose2d(
          Constants.fieldLength - 2.85, Constants.fieldWidth - 6.3, Rotation2d.fromDegrees(130));
  private static final Pose2d blueDepotAlignPose =
      new Pose2d(Constants.blueLine - 0.6, Constants.midLineY, Rotation2d.kZero);
  private static final Pose2d redDepotAlignPose =
      new Pose2d(Constants.redLine + 0.6, Constants.midLineY, Rotation2d.fromDegrees(180.0));
  private static final Translation2d[][] trenchSegments = {
    {Constants.blueTrenchTopInner, Constants.blueTrenchTopOuter},
    {Constants.redTrenchTopOuter, Constants.redTrenchTopInner},
    {Constants.blueTrenchBottomInner, Constants.blueTrenchBottomOuter},
    {Constants.redTrenchBottomOuter, Constants.redTrenchBottomInner}
  };

  private final Drive drive;

  public FieldTargetingService(Drive drive) {
    this.drive = drive;
  }

  public Command autoDriveUnderTrenchCommand(double goalEndVelocity) {
    return drive.autoDriveUnderTrenchCommand(goalEndVelocity);
  }

  public Command driveToOutpostCommand() {
    return drive.driveToOutpostCommand();
  }

  public Command alignToOutpost(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return drive.alignToOutpost(xSupplier, ySupplier);
  }

  public Command alignToDepotCommand() {
    return drive.alignToPose(selectDepotAlignPose(DriverStation.getAlliance()));
  }

  public boolean isRobotNearTrench() {
    return isNearTrench(drive.getPose());
  }

  public Pose2d getOutpostStartPose() {
    return selectCompetitionAutoStartPose(DriverStation.getAlliance(), drive.getPose());
  }

  public Pose2d getOpeningShotPose() {
    return selectOpeningShotPose(DriverStation.getAlliance());
  }

  public Pose2d getEndingShotPose() {
    return selectEndingShotPose(DriverStation.getAlliance());
  }

  public Command driveToOpeningShotCommand() {
    return drive.pathfindToTranslation(getOpeningShotPose().getTranslation());
  }

  static Pose2d selectDepotAlignPose(Optional<Alliance> alliance) {
    return getAlliancePose(alliance, blueDepotAlignPose, redDepotAlignPose);
  }

  static Pose2d selectOutpostStartPose(Optional<Alliance> alliance) {
    return getAlliancePose(alliance, Constants.blueOutpost, Constants.redOutpost);
  }

  static Pose2d selectCompetitionAutoStartPose(Optional<Alliance> alliance, Pose2d currentPose) {
    Pose2d nominalStartPose =
        getAlliancePose(
            alliance, Constants.competitionAutoBlueStart, Constants.competitionAutoRedStart);
    double yOnStartingLine = MathUtil.clamp(currentPose.getY(), 0.0, Constants.fieldWidth);
    return new Pose2d(nominalStartPose.getX(), yOnStartingLine, nominalStartPose.getRotation());
  }

  static Pose2d selectOpeningShotPose(Optional<Alliance> alliance) {
    return getAlliancePose(alliance, blueOutpostOpeningShotPose, redOutpostOpeningShotPose);
  }

  static Pose2d selectEndingShotPose(Optional<Alliance> alliance) {
    return getAlliancePose(alliance, blueOutpostEndingShotPose, redOutpostEndingShotPose);
  }

  static boolean isNearTrench(Pose2d robotPose) {
    Translation2d robotTranslation = robotPose.getTranslation();
    for (Translation2d[] trenchSegment : trenchSegments) {
      if (distanceToSegmentMeters(robotTranslation, trenchSegment[0], trenchSegment[1])
          <= trenchSafetyRetractDistanceMeters) {
        return true;
      }
    }
    return false;
  }

  private static Pose2d getAlliancePose(
      Optional<Alliance> alliance, Pose2d bluePose, Pose2d redPose) {
    return alliance.orElse(Alliance.Blue) == Alliance.Red ? redPose : bluePose;
  }

  private Pose2d getAlliancePose(Pose2d bluePose, Pose2d redPose) {
    return getAlliancePose(DriverStation.getAlliance(), bluePose, redPose);
  }

  private static double distanceToSegmentMeters(
      Translation2d point, Translation2d start, Translation2d end) {
    Translation2d segment = end.minus(start);
    double segmentLengthSquared =
        (segment.getX() * segment.getX()) + (segment.getY() * segment.getY());
    if (segmentLengthSquared <= 1e-9) {
      return point.getDistance(start);
    }

    Translation2d pointOffset = point.minus(start);
    double projection =
        MathUtil.clamp(
            ((pointOffset.getX() * segment.getX()) + (pointOffset.getY() * segment.getY()))
                / segmentLengthSquared,
            0.0,
            1.0);
    Translation2d closestPoint = start.plus(segment.times(projection));
    return point.getDistance(closestPoint);
  }
}
