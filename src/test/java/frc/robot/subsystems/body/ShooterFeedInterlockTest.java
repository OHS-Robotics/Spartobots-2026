package frc.robot.subsystems.body;

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
}
