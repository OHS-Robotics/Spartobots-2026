package frc.robot.testing;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.function.BooleanSupplier;

/** Minimal runner for table-driven simulated command scenarios. */
public final class SimScenarioHarness {
  private SimScenarioHarness() {}

  public record SimScenario<T>(String name, SubsystemFidelity fidelity, T value) {}

  public static void runUntilComplete(
      SimScenario<?> scenario, int maxTicks, Runnable tick, BooleanSupplier isComplete) {
    for (int i = 0; i < maxTicks && !isComplete.getAsBoolean(); i++) {
      tick.run();
    }
    assertTrue(
        isComplete.getAsBoolean(),
        () -> scenario.name() + " did not complete within " + maxTicks + " scheduler ticks");
  }
}
