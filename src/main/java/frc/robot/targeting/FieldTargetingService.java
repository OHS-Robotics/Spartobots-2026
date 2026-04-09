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
import frc.robot.subsystems.drive.DriveConstants;
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
  private static final String blueLowerBumpReturnPathName = "Bump Right In";
  private static final String blueUpperBumpReturnPathName = "Bump Left In";
  private static final double competitionAutoCenterlineBumperMarginMeters = 0.05;
  private static final double competitionAutoPoint1BackMeters = 0.70;
  private static final double competitionAutoPoint1LeftMeters = 0.10;
  private static final double competitionAutoMaxBlueSideCenterX =
      Constants.midLineX
          - (DriveConstants.bumperLengthXMeters / 2.0)
          - competitionAutoCenterlineBumperMarginMeters;
  private static final double competitionAutoMaxLowerLaneCenterY =
      Constants.midLineY
          - (DriveConstants.bumperWidthYMeters / 2.0)
          - competitionAutoCenterlineBumperMarginMeters;
  // Blue-reference points from the annotated auto map. Red variants are derived below.
  private static final Pose2d blueLowerAutoPoint1Pose =
      new Pose2d(
          Constants.blueTrenchBottomInner.getX() - competitionAutoPoint1BackMeters,
          Constants.blueTrenchBottomInner.getY() + competitionAutoPoint1LeftMeters,
          Rotation2d.kZero);
  private static final Pose2d blueUpperAutoPoint1Pose =
      mirrorBluePoseAcrossFieldWidth(blueLowerAutoPoint1Pose);
  private static final Pose2d blueLowerAutoPoint2Pose =
      new Pose2d(competitionAutoMaxBlueSideCenterX, 0.9, Rotation2d.fromDegrees(90.0));
  private static final Pose2d blueUpperAutoPoint2Pose =
      mirrorBluePoseAcrossFieldWidth(blueLowerAutoPoint2Pose);
  private static final Pose2d blueLowerAutoPoint3Pose =
      new Pose2d(
          competitionAutoMaxBlueSideCenterX,
          competitionAutoMaxLowerLaneCenterY,
          Rotation2d.fromDegrees(90.0));
  private static final Pose2d blueUpperAutoPoint3Pose =
      mirrorBluePoseAcrossFieldWidth(blueLowerAutoPoint3Pose);
  private static final Pose2d blueLowerAutoPoint4Pose =
      new Pose2d(5.573116613700566, 2.4890436308262713, Rotation2d.fromDegrees(179.13));
  private static final Pose2d blueUpperAutoPoint4Pose =
      mirrorBluePoseAcrossFieldWidth(blueLowerAutoPoint4Pose);
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

  public CompetitionAutoTargets getCompetitionAutoTargets(double driverStationRetreatMeters) {
    return selectCompetitionAutoTargets(
        DriverStation.getAlliance(), drive.getPose(), driverStationRetreatMeters);
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

  static CompetitionAutoTargets selectCompetitionAutoTargets(
      Optional<Alliance> alliance, Pose2d currentPose, double driverStationRetreatMeters) {
    Pose2d startPose = selectCompetitionAutoStartPose(alliance, currentPose);
    boolean useUpperRouteInBlueReference =
        getBlueReferenceY(alliance, startPose.getY()) > Constants.midLineY;

    Pose2d point1Pose =
        selectAllianceAutoRoutePose(
            alliance,
            useUpperRouteInBlueReference,
            blueLowerAutoPoint1Pose,
            blueUpperAutoPoint1Pose);
    Pose2d point2Pose =
        selectAllianceAutoRoutePose(
            alliance,
            useUpperRouteInBlueReference,
            blueLowerAutoPoint2Pose,
            blueUpperAutoPoint2Pose);
    Pose2d point3Pose =
        selectAllianceAutoRoutePose(
            alliance,
            useUpperRouteInBlueReference,
            blueLowerAutoPoint3Pose,
            blueUpperAutoPoint3Pose);
    Pose2d point4Pose =
        selectAllianceAutoRoutePose(
            alliance,
            useUpperRouteInBlueReference,
            blueLowerAutoPoint4Pose,
            blueUpperAutoPoint4Pose);
    Pose2d point5Pose =
        selectDriverStationRetreatPose(alliance, startPose, driverStationRetreatMeters);
    String bumpReturnPathName =
        useUpperRouteInBlueReference ? blueUpperBumpReturnPathName : blueLowerBumpReturnPathName;

    return new CompetitionAutoTargets(
        startPose, point1Pose, point2Pose, point3Pose, point4Pose, point5Pose, bumpReturnPathName);
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

  private static Pose2d selectAllianceAutoRoutePose(
      Optional<Alliance> alliance,
      boolean useUpperRouteInBlueReference,
      Pose2d blueLowerPose,
      Pose2d blueUpperPose) {
    Pose2d bluePose = useUpperRouteInBlueReference ? blueUpperPose : blueLowerPose;
    return alliance.orElse(Alliance.Blue) == Alliance.Red
        ? mirrorBluePoseForRedAlliance(bluePose)
        : bluePose;
  }

  private static Pose2d selectDriverStationRetreatPose(
      Optional<Alliance> alliance, Pose2d startPose, double retreatMeters) {
    double driverStationDirection = alliance.orElse(Alliance.Blue) == Alliance.Red ? 1.0 : -1.0;
    double retreatedX =
        MathUtil.clamp(
            startPose.getX() + (driverStationDirection * retreatMeters),
            0.0,
            Constants.fieldLength);
    return new Pose2d(retreatedX, startPose.getY(), startPose.getRotation());
  }

  private static double getBlueReferenceY(Optional<Alliance> alliance, double fieldY) {
    return alliance.orElse(Alliance.Blue) == Alliance.Red ? Constants.fieldWidth - fieldY : fieldY;
  }

  private static Pose2d mirrorBluePoseAcrossFieldWidth(Pose2d pose) {
    return new Pose2d(
        pose.getX(),
        Constants.fieldWidth - pose.getY(),
        Rotation2d.fromRadians(-pose.getRotation().getRadians()));
  }

  static Pose2d mirrorBluePoseForRedAlliance(Pose2d pose) {
    return new Pose2d(
        Constants.fieldLength - pose.getX(),
        Constants.fieldWidth - pose.getY(),
        pose.getRotation().minus(Rotation2d.kPi));
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

  public record CompetitionAutoTargets(
      Pose2d startPose,
      Pose2d point1Pose,
      Pose2d point2Pose,
      Pose2d point3Pose,
      Pose2d point4Pose,
      Pose2d point5Pose,
      String bumpReturnPathName) {}
}
