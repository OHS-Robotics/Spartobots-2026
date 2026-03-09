package frc.robot.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj2.command.Commands;
import org.junit.jupiter.api.Test;

class AutoRoutineFactoryTest {
  @Test
  void resetSpecUsesSelectedAutoSpec() {
    AutoSpec spec =
        new AutoSpec(
            AutoSpec.StartZone.UPPER,
            AutoSpec.PreloadPolicy.SCORE,
            AutoSpec.AcquisitionSource.DEPOT,
            1,
            AutoSpec.RiskTier.BALANCED,
            AutoSpec.ParkOption.UPPER);

    assertEquals(
        spec, AutoRoutineFactory.resetSpecFor(AutoOption.forAuto("Upper", spec, Commands::none)));
  }

  @Test
  void resetSpecFallsBackToDefaultForNonAutoOption() {
    assertEquals(
        AutoRoutineFactory.defaultSpec(),
        AutoRoutineFactory.resetSpecFor(AutoOption.forCommand("Do Nothing", Commands::none)));
  }

  @Test
  void initialLibraryHasTenDescriptiveAutos() {
    AutoRoutineFactory factory = new AutoRoutineFactory(null, null, null);

    var options = factory.initialAutoOptions();

    assertEquals(10, options.size());
    assertTrue(options.stream().allMatch(option -> option.name().contains("Start ")));
    assertTrue(options.stream().allMatch(option -> option.name().contains("Band ")));
    assertTrue(options.stream().allMatch(option -> option.name().contains("Risk ")));
    assertTrue(options.stream().allMatch(option -> option.name().contains("Assume ")));
    assertTrue(options.stream().anyMatch(option -> option.name().contains("Outpost Feed")));
    assertTrue(options.stream().anyMatch(option -> option.name().contains("Park First Fallback")));
  }
}
