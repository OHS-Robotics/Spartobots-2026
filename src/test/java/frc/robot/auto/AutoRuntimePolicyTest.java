package frc.robot.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class AutoRuntimePolicyTest {
  @Test
  void safeTierCapsRequestedCyclesAtOne() {
    AutoSpec spec =
        AutoSpec.of(
            AutoSpec.StartZone.CENTER,
            AutoSpec.PreloadPolicy.SCORE,
            AutoSpec.AcquisitionSource.DEPOT,
            2,
            AutoRisk.SAFE,
            AutoSpec.ParkOption.LOWER);

    assertEquals(1, AutoRuntimePolicy.effectiveCycleCount(spec));
  }

  @Test
  void balancedTierOnlyStartsCycleWhenSixSecondsRemain() {
    AutoSpec spec =
        AutoSpec.of(
            AutoSpec.StartZone.CENTER,
            AutoSpec.PreloadPolicy.SCORE,
            AutoSpec.AcquisitionSource.NEUTRAL_FLOOR,
            1,
            AutoRisk.BALANCED,
            AutoSpec.ParkOption.NEAREST);

    assertTrue(AutoRuntimePolicy.shouldStartCycle(spec, 0, 9.0));
    assertFalse(AutoRuntimePolicy.shouldStartCycle(spec, 0, 9.1));
  }

  @Test
  void aggressiveTierOnlyParksWhenReservationWindowRemains() {
    AutoSpec spec =
        AutoSpec.of(
            AutoSpec.StartZone.UPPER,
            AutoSpec.PreloadPolicy.SCORE,
            AutoSpec.AcquisitionSource.DEPOT,
            2,
            AutoRisk.AGGRESSIVE,
            AutoSpec.ParkOption.UPPER);

    assertTrue(AutoRuntimePolicy.canParkNow(spec, 13.5));
    assertFalse(AutoRuntimePolicy.canParkNow(spec, 13.6));
  }
}
