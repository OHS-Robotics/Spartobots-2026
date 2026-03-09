package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import java.util.Objects;
import java.util.function.Supplier;

public final class DriverStationMatchStateProvider implements MatchStateProvider {
  private final Supplier<DriverStationSnapshot> snapshotSupplier;

  public DriverStationMatchStateProvider() {
    this(DriverStationMatchStateProvider::readDriverStationSnapshot);
  }

  DriverStationMatchStateProvider(Supplier<DriverStationSnapshot> snapshotSupplier) {
    this.snapshotSupplier = Objects.requireNonNull(snapshotSupplier, "snapshotSupplier");
  }

  @Override
  public MatchState getMatchState() {
    return MatchStateProvider.fromSnapshot(snapshotSupplier.get());
  }

  private static DriverStationSnapshot readDriverStationSnapshot() {
    return new DriverStationSnapshot(
        DriverStation.getAlliance(),
        DriverStation.getGameSpecificMessage(),
        DriverStation.getMatchTime(),
        DriverStation.isAutonomousEnabled(),
        DriverStation.isTeleopEnabled());
  }
}
