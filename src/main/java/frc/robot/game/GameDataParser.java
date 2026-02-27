package frc.robot.game;

import frc.robot.game.GameStateSubsystem.HubState;

/** Utility methods for parsing game-specific FMS data. */
public final class GameDataParser {
  private GameDataParser() {}

  /**
   * Parses hub state from FMS game data.
   *
   * <p>Returns {@link HubState#INACTIVE} for unknown/invalid input to provide a safe default.
   */
  public static HubState parseHubState(String gameSpecificMessage) {
    HubState strictState = parseHubStateStrict(gameSpecificMessage);
    return strictState == HubState.UNKNOWN ? HubState.INACTIVE : strictState;
  }

  /** Parses hub state from FMS game data without applying fallback defaults. */
  public static HubState parseHubStateStrict(String gameSpecificMessage) {
    String sanitized = sanitize(gameSpecificMessage);
    if (sanitized.isEmpty()) {
      return HubState.UNKNOWN;
    }

    char state = Character.toUpperCase(sanitized.charAt(0));
    return switch (state) {
      case 'A' -> HubState.ACTIVE;
      case 'I' -> HubState.INACTIVE;
      default -> HubState.UNKNOWN;
    };
  }

  private static String sanitize(String gameSpecificMessage) {
    return gameSpecificMessage == null ? "" : gameSpecificMessage.trim();
  }
}
