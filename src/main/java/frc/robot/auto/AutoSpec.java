package frc.robot.auto;

import frc.robot.TargetSelector;
import java.util.Locale;
import java.util.Objects;

public record AutoSpec(
    StartZone startZone,
    PreloadPolicy preloadPolicy,
    AcquisitionSource acquisitionSource,
    int cycleCount,
    RiskTier riskTier,
    ParkOption parkOption) {
  private static final int MAX_CYCLE_COUNT = 2;

  public AutoSpec {
    Objects.requireNonNull(startZone, "startZone");
    Objects.requireNonNull(preloadPolicy, "preloadPolicy");
    Objects.requireNonNull(acquisitionSource, "acquisitionSource");
    Objects.requireNonNull(riskTier, "riskTier");
    Objects.requireNonNull(parkOption, "parkOption");

    cycleCount = Math.max(0, Math.min(MAX_CYCLE_COUNT, cycleCount));
    if (acquisitionSource == AcquisitionSource.NONE || preloadPolicy == PreloadPolicy.HOLD) {
      cycleCount = 0;
    }
  }

  public TargetSelector.AutoStartSelection autoStartSelection() {
    return switch (startZone) {
      case LOWER -> TargetSelector.AutoStartSelection.ALLIANCE_LOWER;
      case CENTER -> TargetSelector.AutoStartSelection.ALLIANCE_CENTER;
      case UPPER -> TargetSelector.AutoStartSelection.ALLIANCE_UPPER;
    };
  }

  public String displayName() {
    String cycleLabel =
        cycleCount == 0 ? "No Cycles" : cycleCount + " Cycle" + (cycleCount == 1 ? "" : "s");
    return String.join(
        " / ",
        startZone.label(),
        preloadPolicy.label(),
        acquisitionSource.label(),
        cycleLabel,
        riskTier.label(),
        parkOption.label());
  }

  public enum StartZone {
    LOWER,
    CENTER,
    UPPER;

    public String label() {
      return titleCase(name());
    }
  }

  public enum PreloadPolicy {
    SCORE,
    HOLD,
    EJECT;

    public String label() {
      return titleCase(name());
    }
  }

  public enum AcquisitionSource {
    DEPOT,
    NEUTRAL_FLOOR,
    NONE;

    public String label() {
      return switch (this) {
        case DEPOT -> "Depot";
        case NEUTRAL_FLOOR -> "Neutral Floor";
        case NONE -> "No Acquire";
      };
    }
  }

  public enum RiskTier {
    SAFE,
    BALANCED,
    AGGRESSIVE;

    public String label() {
      return titleCase(name());
    }
  }

  public enum ParkOption {
    NONE,
    NEAREST,
    LOWER,
    UPPER;

    public String label() {
      return switch (this) {
        case NONE -> "No Park";
        case NEAREST -> "Park Nearest";
        case LOWER -> "Park Lower";
        case UPPER -> "Park Upper";
      };
    }
  }

  private static String titleCase(String value) {
    String normalized = value.toLowerCase(Locale.ROOT).replace('_', ' ');
    String[] words = normalized.split(" ");
    StringBuilder builder = new StringBuilder();
    for (int i = 0; i < words.length; i++) {
      if (i > 0) {
        builder.append(' ');
      }
      if (!words[i].isEmpty()) {
        builder.append(Character.toUpperCase(words[i].charAt(0))).append(words[i].substring(1));
      }
    }
    return builder.toString();
  }
}
