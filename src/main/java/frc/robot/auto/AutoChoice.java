package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Objects;
import java.util.Optional;
import java.util.function.Supplier;

public record AutoChoice(
    String label,
    Optional<AutoSpec> spec,
    Optional<AutoMetadata> metadata,
    Supplier<Command> commandFactory) {
  public AutoChoice {
    Objects.requireNonNull(label, "label");
    spec = spec == null ? Optional.empty() : spec;
    metadata = metadata == null ? Optional.empty() : metadata;
    if (spec.isEmpty()) {
      Objects.requireNonNull(commandFactory, "commandFactory");
    }
  }

  public static AutoChoice forAuto(AutoSpec spec, AutoMetadata metadata) {
    return new AutoChoice(metadata.labelFor(spec), Optional.of(spec), Optional.of(metadata), null);
  }

  public static AutoChoice forCommand(String label, Supplier<Command> commandFactory) {
    return new AutoChoice(label, Optional.empty(), Optional.empty(), commandFactory);
  }

  public AutoCommand compose(AutoRoutineFactory factory) {
    if (spec.isPresent()) {
      return factory.compose(spec.orElseThrow(), metadata.orElseThrow());
    }

    Command command = commandFactory.get();
    return new AutoCommand(
        command == null ? Commands.none() : command.withName(label),
        Optional.empty(),
        Optional.empty());
  }
}
