package frc.robot.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;

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
}
