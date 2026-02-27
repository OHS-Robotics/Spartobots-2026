package frc.robot.game;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.game.GameStateSubsystem.HubState;
import org.junit.jupiter.api.Test;

class GameDataParserTest {
  @Test
  void parsesExplicitHubStateCharacters() {
    assertEquals(HubState.ACTIVE, GameDataParser.parseHubState("A"));
    assertEquals(HubState.ACTIVE, GameDataParser.parseHubState("a"));
    assertEquals(HubState.INACTIVE, GameDataParser.parseHubState("I"));
    assertEquals(HubState.INACTIVE, GameDataParser.parseHubState("i"));
  }

  @Test
  void parseHubStateFallsBackToInactiveForInvalidInput() {
    assertEquals(HubState.INACTIVE, GameDataParser.parseHubState(""));
    assertEquals(HubState.INACTIVE, GameDataParser.parseHubState(" "));
    assertEquals(HubState.INACTIVE, GameDataParser.parseHubState("X"));
    assertEquals(HubState.INACTIVE, GameDataParser.parseHubState(null));
  }

  @Test
  void strictParserReturnsUnknownForInvalidInput() {
    assertEquals(HubState.UNKNOWN, GameDataParser.parseHubStateStrict(""));
    assertEquals(HubState.UNKNOWN, GameDataParser.parseHubStateStrict("unknown"));
    assertEquals(HubState.UNKNOWN, GameDataParser.parseHubStateStrict(null));
  }
}
