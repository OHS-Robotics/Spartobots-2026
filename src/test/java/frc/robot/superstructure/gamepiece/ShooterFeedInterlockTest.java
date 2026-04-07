package frc.robot.superstructure.gamepiece;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class ShooterFeedInterlockTest {
  @Test
  void allowsFeedOnlyWhenAllConditionsAreTrue() {
    assertTrue(ShooterFeedInterlock.shouldAdvanceToShooter(true, true, true));
    assertFalse(ShooterFeedInterlock.shouldAdvanceToShooter(false, true, true));
    assertFalse(ShooterFeedInterlock.shouldAdvanceToShooter(true, false, true));
    assertFalse(ShooterFeedInterlock.shouldAdvanceToShooter(true, true, false));
  }

  @Test
  void manualFeedIndexerPausesOnlyWhenAutoAimShotIsInfeasible() {
    assertTrue(ShooterFeedInterlock.shouldRunIndexerDuringManualFeed(true, false, false));
    assertTrue(ShooterFeedInterlock.shouldRunIndexerDuringManualFeed(true, false, true));
    assertFalse(ShooterFeedInterlock.shouldRunIndexerDuringManualFeed(true, true, false));
    assertTrue(ShooterFeedInterlock.shouldRunIndexerDuringManualFeed(true, true, true));
    assertFalse(ShooterFeedInterlock.shouldRunIndexerDuringManualFeed(false, true, true));
  }
}
