package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.MatchStateProvider;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import org.ironmaple.simulation.gamepieces.GamePiece;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;

public class SpartobotsArena2026Rebuilt extends Arena2026Rebuilt {
  private static final String FUEL_TYPE = "Fuel";
  private static final Translation3d RED_RENDER_POSE = new Translation3d(16.640988, 7.7, 0.844502);
  private static final Translation3d BLUE_RENDER_POSE = new Translation3d(-0.12, 0.325, 0.844502);

  private MatchStateProvider.MatchState matchState =
      new MatchStateProvider.MatchState(
          java.util.Optional.of(Alliance.Blue),
          MatchStateProvider.TowerLetter.NONE,
          MatchStateProvider.Hub.UNKNOWN,
          MatchStateProvider.Hub.UNKNOWN,
          MatchStateProvider.Hub.NONE,
          MatchStateProvider.MatchPhaseScoringContext.DISABLED);
  private double matchTimeSeconds = 0.0;
  private int blueOutpostFuelCount = 24;
  private int redOutpostFuelCount = 24;

  public SpartobotsArena2026Rebuilt(boolean addRampCollider) {
    super(addRampCollider);
    setShouldRunClock(false);
  }

  public void updateMatchContext(
      MatchStateProvider.MatchState matchState, double matchTimeSeconds) {
    this.matchState = matchState;
    this.matchTimeSeconds = matchTimeSeconds;
  }

  @Override
  public boolean isActive(boolean isBlue) {
    MatchStateProvider.Hub activeHub = SimMatchStateUtil.getEffectiveActiveHub(matchState);
    return isBlue
        ? activeHub.matchesAlliance(Alliance.Blue)
        : activeHub.matchesAlliance(Alliance.Red);
  }

  @Override
  public void placeGamePiecesOnField() {
    super.placeGamePiecesOnField();
    blueOutpostFuelCount = 24;
    redOutpostFuelCount = 24;
    syncOutpostBreakdown();
  }

  @Override
  public void simulationSubTick(int tickNum) {
    super.simulationSubTick(tickNum);
    phaseClockPublisher.set(
        SimMatchStateUtil.getPhaseSecondsRemaining(matchState, matchTimeSeconds));
    blueActivePublisher.set(isActive(true));
    redActivePublisher.set(isActive(false));
    syncOutpostBreakdown();
  }

  @Override
  public synchronized List<Pose3d> getGamePiecesPosesByType(String type) {
    List<Pose3d> poses = new ArrayList<>();
    for (GamePiece gamePiece : gamePieces) {
      if (Objects.equals(gamePiece.getType(), type)) {
        poses.add(gamePiece.getPose3d());
      }
    }

    if (Objects.equals(type, FUEL_TYPE)) {
      drawOutpost(poses, true, blueOutpostFuelCount);
      drawOutpost(poses, false, redOutpostFuelCount);
    }

    return poses;
  }

  public boolean consumeOutpostFuel(Alliance alliance) {
    return switch (alliance) {
      case Blue -> {
        if (blueOutpostFuelCount <= 0) {
          yield false;
        }
        blueOutpostFuelCount--;
        syncOutpostBreakdown();
        yield true;
      }
      case Red -> {
        if (redOutpostFuelCount <= 0) {
          yield false;
        }
        redOutpostFuelCount--;
        syncOutpostBreakdown();
        yield true;
      }
    };
  }

  public int getOutpostFuelCount(Alliance alliance) {
    return alliance == Alliance.Red ? redOutpostFuelCount : blueOutpostFuelCount;
  }

  public double getBreakdownValue(Alliance alliance, String key) {
    return alliance == Alliance.Red
        ? redScoringBreakdown.getOrDefault(key, 0.0)
        : blueScoringBreakdown.getOrDefault(key, 0.0);
  }

  private void syncOutpostBreakdown() {
    replaceValueInMatchBreakDown(true, "CurrentFuelInOutpost", blueOutpostFuelCount);
    replaceValueInMatchBreakDown(false, "CurrentFuelInOutpost", redOutpostFuelCount);
  }

  private static void drawOutpost(List<Pose3d> drawList, boolean isBlue, int count) {
    int rendered = 0;
    for (int col = 0; col < 5 && rendered < count; col++) {
      for (int row = 0; row < 5 && rendered < count; row++) {
        rendered++;
        Translation3d base = isBlue ? BLUE_RENDER_POSE : RED_RENDER_POSE;
        Translation3d offset =
            isBlue
                ? new Translation3d(-0.1524 * col, 0.1524 * row, 0.03302 * col)
                : new Translation3d(0.1524 * col, -0.1524 * row, 0.03302 * col);
        drawList.add(new Pose3d(base.plus(offset), new Rotation3d()));
      }
    }
  }
}
