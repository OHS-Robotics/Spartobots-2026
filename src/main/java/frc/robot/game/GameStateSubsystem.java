package frc.robot.game;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.NetworkTablesUtil;
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

  private final NetworkTable subsystemTable = NetworkTablesUtil.subsystemTable("GameState");
  private final NetworkTable telemetryTable = NetworkTablesUtil.telemetryTable(subsystemTable);
  private final NetworkTableEntry rawGameDataEntry = telemetryTable.getEntry("RawGameData");
  private final NetworkTableEntry parsedHubStateEntry = telemetryTable.getEntry("HubState");
  private final NetworkTableEntry hubActiveEntry = telemetryTable.getEntry("HubActive");
  private final NetworkTableEntry gameDataValidEntry = telemetryTable.getEntry("GameDataValid");

  private String rawGameData = "";
  private HubState hubState = HubState.UNKNOWN;
  private HubState lastValidHubState = HubState.UNKNOWN;
  private double lastValidTimestampSeconds = Double.NEGATIVE_INFINITY;
  private boolean gameDataValid = false;

  public GameStateSubsystem() {
    this(DriverStation::getGameSpecificMessage, Timer::getFPGATimestamp);
  }

  GameStateSubsystem(Supplier<String> gameDataSupplier, DoubleSupplier timestampSupplier) {
    this.gameDataSupplier = gameDataSupplier != null ? gameDataSupplier : () -> "";
    this.timestampSupplier = timestampSupplier != null ? timestampSupplier : () -> 0.0;
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
    if (gameDataValid) {
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

    SmartDashboard.putString("GameState/RawGameData", rawGameData);
    SmartDashboard.putString("GameState/HubState", hubState.name());
    SmartDashboard.putBoolean("GameState/HubActive", isHubActive());
    SmartDashboard.putBoolean("GameState/GameDataValid", gameDataValid);

    Logger.recordOutput("GameState/RawGameData", rawGameData);
    Logger.recordOutput("GameState/HubState", hubState.name());
    Logger.recordOutput("GameState/HubActive", isHubActive());
    Logger.recordOutput("GameState/GameDataValid", gameDataValid);
  }

  private static String sanitize(String gameSpecificMessage) {
    return gameSpecificMessage == null ? "" : gameSpecificMessage.trim();
  }
}
