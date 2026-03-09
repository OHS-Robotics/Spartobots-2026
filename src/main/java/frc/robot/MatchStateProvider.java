package frc.robot;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

/**
 * Builds a typed REBUILT match-state view from Driver Station data.
 *
 * <p>The 2026 FMS game-specific message is a single {@code R}/{@code B} selector identifying the
 * alliance whose hub goes inactive first. The rest of the state is derived from that selector, the
 * current alliance, the robot mode, and the Driver Station match clock.
 */
public interface MatchStateProvider {
  double TRANSITION_SHIFT_END_SECONDS = 130.0;
  double SHIFT_1_END_SECONDS = 105.0;
  double SHIFT_2_END_SECONDS = 80.0;
  double SHIFT_3_END_SECONDS = 55.0;
  double SHIFT_4_END_SECONDS = 30.0;

  MatchState getMatchState();

  static MatchState fromSnapshot(DriverStationSnapshot snapshot) {
    GameSpecificData gameSpecificData = GameSpecificData.parse(snapshot.gameSpecificMessage());
    Hub inactiveFirstHub = gameSpecificData.inactiveFirstHub();
    Hub startHub = getStartHub(inactiveFirstHub);
    Hub endHub = getEndHub(inactiveFirstHub);
    MatchPhaseScoringContext scoringContext = getScoringContext(snapshot);
    Hub currentActiveHub = getCurrentActiveHub(scoringContext, startHub, endHub);
    TowerLetter towerLetter = getTowerLetter(scoringContext, currentActiveHub);

    return new MatchState(
        snapshot.alliance(), towerLetter, startHub, endHub, currentActiveHub, scoringContext);
  }

  private static Hub getStartHub(Hub inactiveFirstHub) {
    return switch (inactiveFirstHub) {
      case RED -> Hub.BLUE;
      case BLUE -> Hub.RED;
      default -> Hub.UNKNOWN;
    };
  }

  private static Hub getEndHub(Hub inactiveFirstHub) {
    return switch (inactiveFirstHub) {
      case RED, BLUE -> inactiveFirstHub;
      default -> Hub.UNKNOWN;
    };
  }

  private static MatchPhaseScoringContext getScoringContext(DriverStationSnapshot snapshot) {
    if (snapshot.autonomousEnabled()) {
      return MatchPhaseScoringContext.AUTO;
    }

    if (!snapshot.teleopEnabled()) {
      return MatchPhaseScoringContext.DISABLED;
    }

    double matchTimeSeconds = snapshot.matchTimeSeconds();
    if (!Double.isFinite(matchTimeSeconds) || matchTimeSeconds < 0.0) {
      return MatchPhaseScoringContext.TELEOP_NO_CLOCK;
    }

    if (matchTimeSeconds > TRANSITION_SHIFT_END_SECONDS) {
      return MatchPhaseScoringContext.TRANSITION_SHIFT;
    }
    if (matchTimeSeconds > SHIFT_1_END_SECONDS) {
      return MatchPhaseScoringContext.SHIFT_1;
    }
    if (matchTimeSeconds > SHIFT_2_END_SECONDS) {
      return MatchPhaseScoringContext.SHIFT_2;
    }
    if (matchTimeSeconds > SHIFT_3_END_SECONDS) {
      return MatchPhaseScoringContext.SHIFT_3;
    }
    if (matchTimeSeconds > SHIFT_4_END_SECONDS) {
      return MatchPhaseScoringContext.SHIFT_4;
    }
    return MatchPhaseScoringContext.ENDGAME;
  }

  private static Hub getCurrentActiveHub(
      MatchPhaseScoringContext scoringContext, Hub startHub, Hub endHub) {
    return switch (scoringContext) {
      case AUTO, TRANSITION_SHIFT, ENDGAME -> Hub.BOTH;
      case SHIFT_1, SHIFT_3 -> startHub;
      case SHIFT_2, SHIFT_4 -> endHub;
      case DISABLED -> Hub.NONE;
      case TELEOP_NO_CLOCK -> Hub.UNKNOWN;
    };
  }

  private static TowerLetter getTowerLetter(
      MatchPhaseScoringContext scoringContext, Hub currentActiveHub) {
    return switch (scoringContext) {
      case AUTO -> TowerLetter.A;
      case TRANSITION_SHIFT -> TowerLetter.T;
      case ENDGAME -> TowerLetter.E;
      case SHIFT_1, SHIFT_2, SHIFT_3, SHIFT_4 -> switch (currentActiveHub) {
        case RED -> TowerLetter.R;
        case BLUE -> TowerLetter.B;
        default -> TowerLetter.UNKNOWN;
      };
      case DISABLED -> TowerLetter.NONE;
      case TELEOP_NO_CLOCK -> TowerLetter.UNKNOWN;
    };
  }

  record DriverStationSnapshot(
      Optional<Alliance> alliance,
      String gameSpecificMessage,
      double matchTimeSeconds,
      boolean autonomousEnabled,
      boolean teleopEnabled) {}

  record MatchState(
      Optional<Alliance> alliance,
      TowerLetter towerLetter,
      Hub startHub,
      Hub endHub,
      Hub currentActiveHub,
      MatchPhaseScoringContext scoringContext) {
    public boolean isAllianceHubActive() {
      return alliance.map(currentActiveHub::matchesAlliance).orElse(false);
    }
  }

  enum Hub {
    RED,
    BLUE,
    BOTH,
    NONE,
    UNKNOWN;

    public boolean matchesAlliance(Alliance alliance) {
      return switch (this) {
        case RED -> alliance == Alliance.Red;
        case BLUE -> alliance == Alliance.Blue;
        case BOTH -> true;
        case NONE, UNKNOWN -> false;
      };
    }
  }

  /**
   * Letter shown on the REBUILT team sign/tower display.
   *
   * <p>{@code A/T/R/B/E} correspond to Auto, Transition, Red-active, Blue-active, and End Game.
   */
  enum TowerLetter {
    A("A"),
    T("T"),
    R("R"),
    B("B"),
    E("E"),
    NONE(""),
    UNKNOWN("?");

    private final String displayValue;

    TowerLetter(String displayValue) {
      this.displayValue = displayValue;
    }

    public String displayValue() {
      return displayValue;
    }
  }

  enum MatchPhaseScoringContext {
    DISABLED,
    AUTO,
    TRANSITION_SHIFT,
    SHIFT_1,
    SHIFT_2,
    SHIFT_3,
    SHIFT_4,
    ENDGAME,
    TELEOP_NO_CLOCK
  }
}
