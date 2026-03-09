package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.Objects;
import java.util.Optional;

public record AutoCommand(
    Command command, Optional<AutoSpec> spec, Optional<AutoMetadata> metadata) {
  public AutoCommand {
    Objects.requireNonNull(command, "command");
    spec = spec == null ? Optional.empty() : spec;
    metadata = metadata == null ? Optional.empty() : metadata;
  }
}
