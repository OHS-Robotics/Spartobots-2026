package frc.robot.subsystems.body;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class GamePieceManagerScoringPolicyTest {
  @Test
  void allowActiveAndInactivePolicyAlwaysAllowsScoring() {
    assertTrue(
        GamePieceManager.isScoringAllowed(
            GamePieceManager.ScoringPolicy.ALLOW_ACTIVE_AND_INACTIVE, true));
    assertTrue(
        GamePieceManager.isScoringAllowed(
            GamePieceManager.ScoringPolicy.ALLOW_ACTIVE_AND_INACTIVE, false));
  }

  @Test
  void activeOnlyPolicyBlocksInactiveHub() {
    assertTrue(
        GamePieceManager.isScoringAllowed(GamePieceManager.ScoringPolicy.ACTIVE_ONLY, true));
    assertFalse(
        GamePieceManager.isScoringAllowed(GamePieceManager.ScoringPolicy.ACTIVE_ONLY, false));
  }
}
