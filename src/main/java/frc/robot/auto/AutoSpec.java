package frc.robot.auto;

import frc.robot.TargetSelector;
import java.util.Locale;
import java.util.Objects;

public sealed interface AutoSpec permits AutoSpec.ConfiguredAutoSpec {
  int MAX_CYCLE_COUNT = 2;

  StartZone startZone();

  PreloadPolicy preloadPolicy();

  AcquisitionSource acquisitionSource();

  int cycleCount();

  AutoRisk risk();

  ParkOption parkOption();

  static AutoSpec of(
      StartZone startZone,
      PreloadPolicy preloadPolicy,
      AcquisitionSource acquisitionSource,
      int cycleCount,
      AutoRisk risk,
      ParkOption parkOption) {
    return new ConfiguredAutoSpec(
        startZone, preloadPolicy, acquisitionSource, cycleCount, risk, parkOption);
  }

  default TargetSelector.AutoStartSelection autoStartSelection() {
    return switch (startZone()) {
      case LOWER -> TargetSelector.AutoStartSelection.ALLIANCE_LOWER;
      case CENTER -> TargetSelector.AutoStartSelection.ALLIANCE_CENTER;
      case UPPER -> TargetSelector.AutoStartSelection.ALLIANCE_UPPER;
    };
  }

  default String displayName() {
    String cycleLabel =
        cycleCount() == 0 ? "No Cycles" : cycleCount() + " Cycle" + (cycleCount() == 1 ? "" : "s");
    return String.join(
        " / ",
        startZone().label(),
        preloadPolicy().label(),
        acquisitionSource().label(),
        cycleLabel,
        risk().label(),
        parkOption().label());
  }

  record ConfiguredAutoSpec(
      StartZone startZone,
      PreloadPolicy preloadPolicy,
      AcquisitionSource acquisitionSource,
      int cycleCount,
      AutoRisk risk,
      ParkOption parkOption)
      implements AutoSpec {
    public ConfiguredAutoSpec {
      Objects.requireNonNull(startZone, "startZone");
      Objects.requireNonNull(preloadPolicy, "preloadPolicy");
      Objects.requireNonNull(acquisitionSource, "acquisitionSource");
      Objects.requireNonNull(risk, "risk");
      Objects.requireNonNull(parkOption, "parkOption");

      cycleCount = Math.max(0, Math.min(MAX_CYCLE_COUNT, cycleCount));
      if (acquisitionSource == AcquisitionSource.NONE || preloadPolicy == PreloadPolicy.HOLD) {
        cycleCount = 0;
      }
    }
  }

  enum StartZone {
    LOWER,
    CENTER,
    UPPER;

    public String label() {
      return titleCase(name());
    }
  }

  enum PreloadPolicy {
    SCORE,
    HOLD,
    EJECT;

    public String label() {
      return titleCase(name());
    }
  }

  enum AcquisitionSource {
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

  enum ParkOption {
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
