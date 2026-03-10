package frc.robot.testing;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import java.util.function.Consumer;
import java.util.function.Supplier;

/** Shared assertions for basic mechanism goal/state contracts. */
public final class SubsystemContractSuite {
  private SubsystemContractSuite() {}

  public interface GoalController<G, S> {
    void setGoal(G goal);

    G getGoal();

    S getStatus();

    boolean isAtGoal();

    void stop();
  }

  public static <G, S> void verifyGoalLifecycle(
      String name,
      SubsystemFidelity fidelity,
      GoalController<G, S> controller,
      Supplier<Boolean> statusAtGoalSupplier,
      G commandedGoal,
      G stoppedGoal,
      Consumer<S> commandedStatusAssertions,
      Consumer<S> stoppedStatusAssertions) {
    assertNotNull(name, "name");
    assertNotNull(fidelity, "fidelity");
    assertNotNull(controller, "controller");
    assertNotNull(statusAtGoalSupplier, "statusAtGoalSupplier");
    assertNotNull(commandedStatusAssertions, "commandedStatusAssertions");
    assertNotNull(stoppedStatusAssertions, "stoppedStatusAssertions");

    controller.setGoal(commandedGoal);
    assertEquals(
        commandedGoal, controller.getGoal(), name + " should echo the last commanded goal");

    S commandedStatus = controller.getStatus();
    assertNotNull(commandedStatus, name + " should always return a status object");
    assertEquals(
        controller.isAtGoal(),
        statusAtGoalSupplier.get(),
        name + " should keep isAtGoal consistent with status");
    commandedStatusAssertions.accept(commandedStatus);

    controller.stop();
    assertEquals(stoppedGoal, controller.getGoal(), name + " stop() should move to its safe goal");

    S stoppedStatus = controller.getStatus();
    assertNotNull(stoppedStatus, name + " should still return status after stop()");
    assertEquals(
        controller.isAtGoal(),
        statusAtGoalSupplier.get(),
        name + " should keep isAtGoal consistent after stop()");
    stoppedStatusAssertions.accept(stoppedStatus);
  }
}
