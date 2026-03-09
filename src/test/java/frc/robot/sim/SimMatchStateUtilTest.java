package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldTargets;
import frc.robot.MatchStateProvider;
import frc.robot.TargetSelector;
import java.util.Optional;
import org.junit.jupiter.api.Test;

class SimMatchStateUtilTest {
  @Test
  void fallsBackToAllianceHubDuringShiftOneWithoutGameData() {
    MatchStateProvider.MatchState matchState =
        new MatchStateProvider.MatchState(
            Optional.of(Alliance.Blue),
            MatchStateProvider.TowerLetter.UNKNOWN,
            MatchStateProvider.Hub.UNKNOWN,
            MatchStateProvider.Hub.UNKNOWN,
            MatchStateProvider.Hub.UNKNOWN,
            MatchStateProvider.MatchPhaseScoringContext.SHIFT_1);

    assertEquals(MatchStateProvider.Hub.BLUE, SimMatchStateUtil.getEffectiveActiveHub(matchState));
    assertEquals(
        FieldTargets.HUB.bluePose().getX(),
        SimMatchStateUtil.getHubPose(TargetSelector.HubSelection.ACTIVE, Alliance.Blue, matchState)
            .getX(),
        1e-9);
  }

  @Test
  void flipsToOpponentHubDuringShiftTwoWithoutGameData() {
    MatchStateProvider.MatchState matchState =
        new MatchStateProvider.MatchState(
            Optional.of(Alliance.Blue),
            MatchStateProvider.TowerLetter.UNKNOWN,
            MatchStateProvider.Hub.UNKNOWN,
            MatchStateProvider.Hub.UNKNOWN,
            MatchStateProvider.Hub.UNKNOWN,
            MatchStateProvider.MatchPhaseScoringContext.SHIFT_2);

    assertEquals(MatchStateProvider.Hub.RED, SimMatchStateUtil.getEffectiveActiveHub(matchState));
    assertEquals(
        FieldTargets.HUB.redPose().getX(),
        SimMatchStateUtil.getHubPose(TargetSelector.HubSelection.ACTIVE, Alliance.Blue, matchState)
            .getX(),
        1e-9);
  }

  @Test
  void computesPhaseSecondsRemainingFromMatchClock() {
    MatchStateProvider.MatchState matchState =
        new MatchStateProvider.MatchState(
            Optional.of(Alliance.Red),
            MatchStateProvider.TowerLetter.R,
            MatchStateProvider.Hub.RED,
            MatchStateProvider.Hub.BLUE,
            MatchStateProvider.Hub.RED,
            MatchStateProvider.MatchPhaseScoringContext.SHIFT_3);

    assertEquals(12.0, SimMatchStateUtil.getPhaseSecondsRemaining(matchState, 67.0), 1e-9);
  }
}
