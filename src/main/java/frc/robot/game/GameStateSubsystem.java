package frc.robot.game;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.NetworkTablesUtil;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Tracks FMS-provided game state data (active vs inactive hub). */
public class GameStateSubsystem extends SubsystemBase {
  public enum HubState {
    ACTIVE,
    INACTIVE,
    UNKNOWN
  }

  private static final double invalidDataHoldSeconds = 1.0;

  private final Supplier<String> gameDataSupplier;
  private final DoubleSupplier timestampSupplier;
  private final BooleanSupplier autonomousEnabledSupplier;

  private final NetworkTable gameStateTable = NetworkTablesUtil.state("Game");
  private final NetworkTableEntry rawGameDataEntry = gameStateTable.getEntry("RawGameData");
  private final NetworkTableEntry parsedHubStateEntry = gameStateTable.getEntry("HubState");
  private final NetworkTableEntry hubActiveEntry = gameStateTable.getEntry("HubActive");
  private final NetworkTableEntry gameDataValidEntry = gameStateTable.getEntry("GameDataValid");

  private String rawGameData = "";
  private HubState hubState = HubState.UNKNOWN;
  private HubState lastValidHubState = HubState.UNKNOWN;
  private double lastValidTimestampSeconds = Double.NEGATIVE_INFINITY;
  private boolean gameDataValid = false;

  public GameStateSubsystem() {
    this(
        DriverStation::getGameSpecificMessage,
        Timer::getFPGATimestamp,
        DriverStation::isAutonomousEnabled);
  }

  GameStateSubsystem(Supplier<String> gameDataSupplier, DoubleSupplier timestampSupplier) {
    this(gameDataSupplier, timestampSupplier, () -> false);
  }

  GameStateSubsystem(
      Supplier<String> gameDataSupplier,
      DoubleSupplier timestampSupplier,
      BooleanSupplier autonomousEnabledSupplier) {
    this.gameDataSupplier = gameDataSupplier != null ? gameDataSupplier : () -> "";
    this.timestampSupplier = timestampSupplier != null ? timestampSupplier : () -> 0.0;
    this.autonomousEnabledSupplier =
        autonomousEnabledSupplier != null ? autonomousEnabledSupplier : () -> false;
  }

  @Override
  public void periodic() {
    updateStateFromFms();
  }

  public HubState getHubState() {
    return hubState;
  }

  public boolean isHubActive() {
    return hubState == HubState.ACTIVE;
  }

  public String getRawGameData() {
    return rawGameData;
  }

  public boolean isGameDataValid() {
    return gameDataValid;
  }

  private void updateStateFromFms() {
    rawGameData = sanitize(gameDataSupplier.get());
    HubState parsedHubState = GameDataParser.parseHubStateStrict(rawGameData);
    double nowSeconds = timestampSupplier.getAsDouble();

    gameDataValid = parsedHubState != HubState.UNKNOWN;
    if (autonomousEnabledSupplier.getAsBoolean()) {
      hubState = HubState.ACTIVE;
    } else if (gameDataValid) {
      hubState = parsedHubState;
      lastValidHubState = parsedHubState;
      lastValidTimestampSeconds = nowSeconds;
    } else {
      boolean withinHoldWindow =
          lastValidHubState != HubState.UNKNOWN
              && Double.isFinite(lastValidTimestampSeconds)
              && (nowSeconds - lastValidTimestampSeconds) <= invalidDataHoldSeconds;
      hubState = withinHoldWindow ? lastValidHubState : HubState.INACTIVE;
    }

    publishOutputs();
  }

  private void publishOutputs() {
    rawGameDataEntry.setString(rawGameData);
    parsedHubStateEntry.setString(hubState.name());
    hubActiveEntry.setBoolean(isHubActive());
    gameDataValidEntry.setBoolean(gameDataValid);

    Logger.recordOutput(NetworkTablesUtil.logPath("Game/State/RawGameData"), rawGameData);
    Logger.recordOutput(NetworkTablesUtil.logPath("Game/State/HubState"), hubState.name());
    Logger.recordOutput(NetworkTablesUtil.logPath("Game/State/HubActive"), isHubActive());
    Logger.recordOutput(NetworkTablesUtil.logPath("Game/State/GameDataValid"), gameDataValid);
  }

  private static String sanitize(String gameSpecificMessage) {
    return gameSpecificMessage == null ? "" : gameSpecificMessage.trim();
  }
}
