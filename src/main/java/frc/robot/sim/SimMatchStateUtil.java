package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldTargets;
import frc.robot.MatchStateProvider;
import frc.robot.TargetSelector;

public final class SimMatchStateUtil {
  private static final double TRANSITION_SHIFT_END_SECONDS = 130.0;
  private static final double SHIFT_1_END_SECONDS = 105.0;
  private static final double SHIFT_2_END_SECONDS = 80.0;
  private static final double SHIFT_3_END_SECONDS = 55.0;
  private static final double SHIFT_4_END_SECONDS = 30.0;

  private SimMatchStateUtil() {}

  public static MatchStateProvider.Hub getEffectiveActiveHub(
      MatchStateProvider.MatchState matchState) {
    if (matchState.currentActiveHub() != MatchStateProvider.Hub.UNKNOWN) {
      return matchState.currentActiveHub();
    }

    Alliance alliance = matchState.alliance().orElse(Alliance.Blue);
    return switch (matchState.scoringContext()) {
      case AUTO, TRANSITION_SHIFT, ENDGAME -> MatchStateProvider.Hub.BOTH;
      case SHIFT_1, SHIFT_3 -> alliance == Alliance.Red
          ? MatchStateProvider.Hub.RED
          : MatchStateProvider.Hub.BLUE;
      case SHIFT_2, SHIFT_4 -> alliance == Alliance.Red
          ? MatchStateProvider.Hub.BLUE
          : MatchStateProvider.Hub.RED;
      case DISABLED -> MatchStateProvider.Hub.NONE;
      case TELEOP_NO_CLOCK -> MatchStateProvider.Hub.BOTH;
    };
  }

  public static Pose3d getHubPose(
      TargetSelector.HubSelection selection,
      Alliance alliance,
      MatchStateProvider.MatchState matchState) {
    return switch (selection) {
      case ALLIANCE -> FieldTargets.HUB.forAlliance(alliance);
      case OPPONENT -> FieldTargets.HUB.forOpponent(alliance);
      case ACTIVE -> switch (getEffectiveActiveHub(matchState)) {
        case BLUE -> FieldTargets.HUB.bluePose();
        case RED -> FieldTargets.HUB.redPose();
        case BOTH, NONE, UNKNOWN -> FieldTargets.HUB.forAlliance(alliance);
      };
    };
  }

  public static double getPhaseSecondsRemaining(
      MatchStateProvider.MatchState matchState, double matchTimeSeconds) {
    if (!Double.isFinite(matchTimeSeconds)) {
      return 0.0;
    }

    return Math.max(
        0.0,
        switch (matchState.scoringContext()) {
          case AUTO, ENDGAME -> matchTimeSeconds;
          case TRANSITION_SHIFT -> matchTimeSeconds - TRANSITION_SHIFT_END_SECONDS;
          case SHIFT_1 -> matchTimeSeconds - SHIFT_1_END_SECONDS;
          case SHIFT_2 -> matchTimeSeconds - SHIFT_2_END_SECONDS;
          case SHIFT_3 -> matchTimeSeconds - SHIFT_3_END_SECONDS;
          case SHIFT_4 -> matchTimeSeconds - SHIFT_4_END_SECONDS;
          case DISABLED, TELEOP_NO_CLOCK -> 0.0;
        });
  }
}
