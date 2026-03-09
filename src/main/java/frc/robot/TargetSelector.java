package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class TargetSelector {
  private static final MatchStateProvider MATCH_STATE_PROVIDER =
      new DriverStationMatchStateProvider();

  private TargetSelector() {}

  public static Pose2d getHubPose(HubSelection selection) {
    return getHubPose3d(selection).toPose2d();
  }

  public static Pose3d getHubPose3d(HubSelection selection) {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    return getHubPose3d(selection, alliance, MATCH_STATE_PROVIDER.getMatchState());
  }

  static Pose3d getHubPose3d(
      HubSelection selection, Alliance alliance, MatchStateProvider.MatchState matchState) {
    return switch (selection) {
      case ALLIANCE -> FieldTargets.HUB.forAlliance(alliance);
      case OPPONENT -> FieldTargets.HUB.forOpponent(alliance);
      case ACTIVE -> switch (matchState.currentActiveHub()) {
        case BLUE -> FieldTargets.HUB.bluePose();
        case RED -> FieldTargets.HUB.redPose();
        case BOTH, NONE, UNKNOWN -> FieldTargets.HUB.forAlliance(alliance);
      };
    };
  }

  public static Pose2d getOutpostPose(OutpostSelection selection) {
    return getOutpostPose(selection, DriverStation.getAlliance().orElse(Alliance.Blue));
  }

  static Pose2d getOutpostPose(OutpostSelection selection, Alliance alliance) {
    return switch (selection) {
      case ALLIANCE -> FieldTargets.OUTPOST.forAlliance(alliance);
      case OPPONENT -> FieldTargets.OUTPOST.forOpponent(alliance);
    };
  }

  public static FieldTargets.FieldZone getIntakeZone(IntakeZoneSelection selection) {
    return getIntakeZone(selection, DriverStation.getAlliance().orElse(Alliance.Blue));
  }

  static FieldTargets.FieldZone getIntakeZone(IntakeZoneSelection selection, Alliance alliance) {
    return switch (selection) {
      case ALLIANCE_DEPOT -> FieldTargets.DEPOT_INTAKE.forAlliance(alliance);
      case OPPONENT_DEPOT -> FieldTargets.DEPOT_INTAKE.forOpponent(alliance);
      case NEUTRAL_FLOOR -> FieldTargets.NEUTRAL_FLOOR_INTAKE;
    };
  }

  public static FieldTargets.FieldZone getParkZone(ParkZoneSelection selection) {
    return getParkZone(selection, DriverStation.getAlliance().orElse(Alliance.Blue));
  }

  static FieldTargets.FieldZone getParkZone(ParkZoneSelection selection, Alliance alliance) {
    return switch (selection) {
      case ALLIANCE_LOWER -> FieldTargets.LOWER_PARK_ZONE.forAlliance(alliance);
      case ALLIANCE_UPPER -> FieldTargets.UPPER_PARK_ZONE.forAlliance(alliance);
      case OPPONENT_LOWER -> FieldTargets.LOWER_PARK_ZONE.forOpponent(alliance);
      case OPPONENT_UPPER -> FieldTargets.UPPER_PARK_ZONE.forOpponent(alliance);
    };
  }

  public static Pose2d getAutoStartPose(AutoStartSelection selection) {
    return getAutoStartPose(selection, DriverStation.getAlliance().orElse(Alliance.Blue));
  }

  static Pose2d getAutoStartPose(AutoStartSelection selection, Alliance alliance) {
    return switch (selection) {
      case ALLIANCE_LOWER -> FieldTargets.AUTO_START_LOWER.forAlliance(alliance);
      case ALLIANCE_CENTER -> FieldTargets.AUTO_START_CENTER.forAlliance(alliance);
      case ALLIANCE_UPPER -> FieldTargets.AUTO_START_UPPER.forAlliance(alliance);
      case OPPONENT_LOWER -> FieldTargets.AUTO_START_LOWER.forOpponent(alliance);
      case OPPONENT_CENTER -> FieldTargets.AUTO_START_CENTER.forOpponent(alliance);
      case OPPONENT_UPPER -> FieldTargets.AUTO_START_UPPER.forOpponent(alliance);
    };
  }

  public static enum HubSelection {
    ALLIANCE,
    OPPONENT,
    ACTIVE
  }

  public static enum IntakeZoneSelection {
    ALLIANCE_DEPOT,
    OPPONENT_DEPOT,
    NEUTRAL_FLOOR
  }

  public static enum OutpostSelection {
    ALLIANCE,
    OPPONENT
  }

  public static enum ParkZoneSelection {
    ALLIANCE_LOWER,
    ALLIANCE_UPPER,
    OPPONENT_LOWER,
    OPPONENT_UPPER
  }

  public static enum AutoStartSelection {
    ALLIANCE_LOWER,
    ALLIANCE_CENTER,
    ALLIANCE_UPPER,
    OPPONENT_LOWER,
    OPPONENT_CENTER,
    OPPONENT_UPPER
  }
}
