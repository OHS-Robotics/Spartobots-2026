package frc.robot.game;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.game.GameStateSubsystem.HubState;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.Test;

class GameStateSubsystemTest {
  @Test
  void holdsLastValidHubStateForOneSecondThenFallsBackToInactive() {
    AtomicReference<String> rawGameData = new AtomicReference<>("");
    double[] nowSeconds = new double[] {0.0};
    GameStateSubsystem subsystem = new GameStateSubsystem(rawGameData::get, () -> nowSeconds[0]);

    subsystem.periodic();
    assertEquals(HubState.INACTIVE, subsystem.getHubState());

    rawGameData.set("A");
    subsystem.periodic();
    assertEquals(HubState.ACTIVE, subsystem.getHubState());

    rawGameData.set("?");
    nowSeconds[0] = 0.9;
    subsystem.periodic();
    assertEquals(HubState.ACTIVE, subsystem.getHubState());

    nowSeconds[0] = 1.1;
    subsystem.periodic();
    assertEquals(HubState.INACTIVE, subsystem.getHubState());
  }
}
