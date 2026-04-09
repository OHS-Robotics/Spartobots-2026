package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class RobotContainerTest {
  @Test
  void autoShotControlSuppressesTrenchSafetyRetract() {
    assertFalse(RobotContainer.shouldApplyTrenchSafetyRetract(true, true, true, false));
  }

  @Test
  void teleopAutoAimSuppressesTrenchSafetyRetract() {
    assertFalse(RobotContainer.shouldApplyTrenchSafetyRetract(true, false, true, true));
  }

  @Test
  void trenchSafetyRetractStillAppliesOutsideAutoShotControl() {
    assertTrue(RobotContainer.shouldApplyTrenchSafetyRetract(true, true, false, false));
    assertTrue(RobotContainer.shouldApplyTrenchSafetyRetract(true, false, true, false));
    assertFalse(RobotContainer.shouldApplyTrenchSafetyRetract(false, true, true, false));
  }
}
