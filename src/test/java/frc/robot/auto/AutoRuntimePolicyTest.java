package frc.robot.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class AutoRuntimePolicyTest {
  @Test
  void safeTierCapsRequestedCyclesAtOne() {
    AutoSpec spec =
        new AutoSpec(
            AutoSpec.StartZone.CENTER,
            AutoSpec.PreloadPolicy.SCORE,
            AutoSpec.AcquisitionSource.DEPOT,
            2,
            AutoSpec.RiskTier.SAFE,
            AutoSpec.ParkOption.LOWER);

    assertEquals(1, AutoRuntimePolicy.effectiveCycleCount(spec));
  }

  @Test
  void balancedTierOnlyStartsCycleWhenSixSecondsRemain() {
    AutoSpec spec =
        new AutoSpec(
            AutoSpec.StartZone.CENTER,
            AutoSpec.PreloadPolicy.SCORE,
            AutoSpec.AcquisitionSource.NEUTRAL_FLOOR,
            1,
            AutoSpec.RiskTier.BALANCED,
            AutoSpec.ParkOption.NEAREST);

    assertTrue(AutoRuntimePolicy.shouldStartCycle(spec, 0, 9.0));
    assertFalse(AutoRuntimePolicy.shouldStartCycle(spec, 0, 9.1));
  }

  @Test
  void aggressiveTierOnlyParksWhenReservationWindowRemains() {
    AutoSpec spec =
        new AutoSpec(
            AutoSpec.StartZone.UPPER,
            AutoSpec.PreloadPolicy.SCORE,
            AutoSpec.AcquisitionSource.DEPOT,
            2,
            AutoSpec.RiskTier.AGGRESSIVE,
            AutoSpec.ParkOption.UPPER);

    assertTrue(AutoRuntimePolicy.shouldAttemptPark(spec, 13.5));
    assertFalse(AutoRuntimePolicy.shouldAttemptPark(spec, 13.6));
  }
}
