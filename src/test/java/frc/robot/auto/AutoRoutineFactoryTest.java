package frc.robot.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class AutoRoutineFactoryTest {
  @Test
  void resetSpecUsesSelectedAutoSpec() {
    AutoSpec spec =
        AutoSpec.of(
            AutoSpec.StartZone.UPPER,
            AutoSpec.PreloadPolicy.SCORE,
            AutoSpec.AcquisitionSource.DEPOT,
            1,
            AutoRisk.BALANCED,
            AutoSpec.ParkOption.UPPER);

    assertEquals(
        spec,
        AutoRoutineFactory.resetSpecFor(
            AutoChoice.forAuto(
                spec,
                new AutoMetadata(
                    "Upper",
                    frc.robot.RobotAction.ACQUIRE_DEPOT,
                    AutoExpectation.SINGLE_CYCLE,
                    ""))));
  }

  @Test
  void resetSpecFallsBackToDefaultForNonAutoChoice() {
    assertEquals(
        AutoRoutineFactory.defaultSpec(),
        AutoRoutineFactory.resetSpecFor(AutoChoice.forCommand("Do Nothing", () -> null)));
  }

  @Test
  void initialLibraryHasTenDescriptiveAutos() {
    AutoRoutineFactory factory = new AutoRoutineFactory(null, null, null);

    var options = factory.initialAutoChoices();

    assertEquals(10, options.size());
    assertTrue(options.stream().allMatch(option -> option.label().contains("Start ")));
    assertTrue(options.stream().allMatch(option -> option.label().contains("Band ")));
    assertTrue(options.stream().allMatch(option -> option.label().contains("Risk ")));
    assertTrue(options.stream().allMatch(option -> option.label().contains("Assume ")));
    assertTrue(options.stream().anyMatch(option -> option.label().contains("Outpost Feed")));
    assertTrue(options.stream().anyMatch(option -> option.label().contains("Park First Fallback")));
  }
}
