package frc.robot.auto;

import java.util.Locale;

public enum AutoRisk {
  SAFE,
  BALANCED,
  AGGRESSIVE;

  public String label() {
    String normalized = name().toLowerCase(Locale.ROOT);
    return Character.toUpperCase(normalized.charAt(0)) + normalized.substring(1);
  }
}
