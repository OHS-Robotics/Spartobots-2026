package frc.robot;

import java.util.Objects;

public sealed interface GameSpecificData permits GameSpecificData.RebuiltGameSpecificData {
  MatchStateProvider.Hub inactiveFirstHub();

  default boolean isKnown() {
    return inactiveFirstHub() != MatchStateProvider.Hub.UNKNOWN;
  }

  static GameSpecificData parse(String rawMessage) {
    if (rawMessage == null) {
      return new RebuiltGameSpecificData(MatchStateProvider.Hub.UNKNOWN);
    }

    String trimmedMessage = rawMessage.trim();
    if (trimmedMessage.isEmpty()) {
      return new RebuiltGameSpecificData(MatchStateProvider.Hub.UNKNOWN);
    }

    MatchStateProvider.Hub inactiveFirstHub =
        switch (Character.toUpperCase(trimmedMessage.charAt(0))) {
          case 'R' -> MatchStateProvider.Hub.RED;
          case 'B' -> MatchStateProvider.Hub.BLUE;
          default -> MatchStateProvider.Hub.UNKNOWN;
        };
    return new RebuiltGameSpecificData(inactiveFirstHub);
  }

  record RebuiltGameSpecificData(MatchStateProvider.Hub inactiveFirstHub)
      implements GameSpecificData {
    public RebuiltGameSpecificData {
      Objects.requireNonNull(inactiveFirstHub, "inactiveFirstHub");
    }
  }
}
