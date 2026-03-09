package frc.robot.auto;

import frc.robot.RobotAction;
import java.util.Objects;

public record AutoMetadata(
    String family, RobotAction primaryAction, AutoExpectation expectation, String assumptions) {
  public AutoMetadata {
    Objects.requireNonNull(family, "family");
    Objects.requireNonNull(primaryAction, "primaryAction");
    Objects.requireNonNull(expectation, "expectation");
    Objects.requireNonNull(assumptions, "assumptions");
  }

  public String labelFor(AutoSpec spec) {
    return String.format(
        "%s | Start %s | Band %s | Risk %s | Assume %s",
        family,
        spec.startZone().label(),
        expectation.pointsBand(),
        spec.risk().label(),
        assumptions);
  }
}
