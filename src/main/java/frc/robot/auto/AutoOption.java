package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.Objects;
import java.util.Optional;
import java.util.function.Supplier;

public record AutoOption(String name, Supplier<Command> commandFactory, Optional<AutoSpec> spec) {
  public AutoOption {
    Objects.requireNonNull(name, "name");
    Objects.requireNonNull(commandFactory, "commandFactory");
    spec = spec == null ? Optional.empty() : spec;
  }

  public static AutoOption forAuto(String name, AutoSpec spec, Supplier<Command> commandFactory) {
    return new AutoOption(name, commandFactory, Optional.of(spec));
  }

  public static AutoOption forCommand(String name, Supplier<Command> commandFactory) {
    return new AutoOption(name, commandFactory, Optional.empty());
  }

  public Command buildCommand() {
    Command command = commandFactory.get();
    return command == null ? null : command.withName(name);
  }
}
