package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;
import org.junit.jupiter.api.Test;

class MatchStateProviderTest {
  @Test
  void derivesShiftOneStateWhenRedHubGoesInactiveFirst() {
    MatchStateProvider.MatchState matchState =
        MatchStateProvider.fromSnapshot(
            new MatchStateProvider.DriverStationSnapshot(
                Optional.of(Alliance.Blue), "R", 120.0, false, true));

    assertEquals(MatchStateProvider.TowerLetter.B, matchState.towerLetter());
    assertEquals(MatchStateProvider.Hub.BLUE, matchState.startHub());
    assertEquals(MatchStateProvider.Hub.RED, matchState.endHub());
    assertEquals(MatchStateProvider.Hub.BLUE, matchState.currentActiveHub());
    assertEquals(MatchStateProvider.MatchPhaseScoringContext.SHIFT_1, matchState.scoringContext());
    assertTrue(matchState.isAllianceHubActive());
  }

  @Test
  void keepsBothHubsActiveInAuto() {
    MatchStateProvider.MatchState matchState =
        MatchStateProvider.fromSnapshot(
            new MatchStateProvider.DriverStationSnapshot(
                Optional.of(Alliance.Red), "b", 18.0, true, false));

    assertEquals(MatchStateProvider.TowerLetter.A, matchState.towerLetter());
    assertEquals(MatchStateProvider.Hub.RED, matchState.startHub());
    assertEquals(MatchStateProvider.Hub.BLUE, matchState.endHub());
    assertEquals(MatchStateProvider.Hub.BOTH, matchState.currentActiveHub());
    assertEquals(MatchStateProvider.MatchPhaseScoringContext.AUTO, matchState.scoringContext());
    assertTrue(matchState.isAllianceHubActive());
  }

  @Test
  void reportsUnknownShiftStateWithoutValidGameData() {
    MatchStateProvider.MatchState matchState =
        MatchStateProvider.fromSnapshot(
            new MatchStateProvider.DriverStationSnapshot(
                Optional.of(Alliance.Red), "", 90.0, false, true));

    assertEquals(MatchStateProvider.TowerLetter.UNKNOWN, matchState.towerLetter());
    assertEquals(MatchStateProvider.Hub.UNKNOWN, matchState.startHub());
    assertEquals(MatchStateProvider.Hub.UNKNOWN, matchState.endHub());
    assertEquals(MatchStateProvider.Hub.UNKNOWN, matchState.currentActiveHub());
    assertEquals(MatchStateProvider.MatchPhaseScoringContext.SHIFT_2, matchState.scoringContext());
    assertFalse(matchState.isAllianceHubActive());
  }

  @Test
  void reportsNoActiveHubWhileDisabled() {
    MatchStateProvider.MatchState matchState =
        MatchStateProvider.fromSnapshot(
            new MatchStateProvider.DriverStationSnapshot(
                Optional.of(Alliance.Blue), "B", 0.0, false, false));

    assertEquals(MatchStateProvider.TowerLetter.NONE, matchState.towerLetter());
    assertEquals(MatchStateProvider.Hub.RED, matchState.startHub());
    assertEquals(MatchStateProvider.Hub.BLUE, matchState.endHub());
    assertEquals(MatchStateProvider.Hub.NONE, matchState.currentActiveHub());
    assertEquals(MatchStateProvider.MatchPhaseScoringContext.DISABLED, matchState.scoringContext());
    assertFalse(matchState.isAllianceHubActive());
  }
}
