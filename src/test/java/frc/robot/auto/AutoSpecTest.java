package frc.robot.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.TargetSelector;
import org.junit.jupiter.api.Test;

class AutoSpecTest {
  @Test
  void holdPreloadForcesCycleCountToZero() {
    AutoSpec spec =
        AutoSpec.of(
            AutoSpec.StartZone.CENTER,
            AutoSpec.PreloadPolicy.HOLD,
            AutoSpec.AcquisitionSource.DEPOT,
            2,
            AutoRisk.BALANCED,
            AutoSpec.ParkOption.NEAREST);

    assertEquals(0, spec.cycleCount());
  }

  @Test
  void noAcquisitionForcesCycleCountToZero() {
    AutoSpec spec =
        AutoSpec.of(
            AutoSpec.StartZone.CENTER,
            AutoSpec.PreloadPolicy.SCORE,
            AutoSpec.AcquisitionSource.NONE,
            2,
            AutoRisk.BALANCED,
            AutoSpec.ParkOption.NEAREST);

    assertEquals(0, spec.cycleCount());
  }

  @Test
  void cycleCountClampsToV1Maximum() {
    AutoSpec spec =
        AutoSpec.of(
            AutoSpec.StartZone.UPPER,
            AutoSpec.PreloadPolicy.SCORE,
            AutoSpec.AcquisitionSource.DEPOT,
            99,
            AutoRisk.AGGRESSIVE,
            AutoSpec.ParkOption.UPPER);

    assertEquals(2, spec.cycleCount());
  }

  @Test
  void startZoneMapsToAllianceAutoStartSelection() {
    AutoSpec spec =
        AutoSpec.of(
            AutoSpec.StartZone.LOWER,
            AutoSpec.PreloadPolicy.SCORE,
            AutoSpec.AcquisitionSource.DEPOT,
            1,
            AutoRisk.SAFE,
            AutoSpec.ParkOption.LOWER);

    assertEquals(TargetSelector.AutoStartSelection.ALLIANCE_LOWER, spec.autoStartSelection());
  }
}
