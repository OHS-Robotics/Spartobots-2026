package frc.robot.auto;

public final class AutoRuntimePolicy {
  static final double AUTO_PERIOD_SECONDS = 15.0;

  private AutoRuntimePolicy() {}

  public static int effectiveCycleCount(AutoSpec spec) {
    return Math.min(spec.cycleCount(), cycleCap(spec.riskTier()));
  }

  public static double speedScale(AutoSpec.RiskTier riskTier) {
    return switch (riskTier) {
      case SAFE -> 0.55;
      case BALANCED -> 0.75;
      case AGGRESSIVE -> 1.0;
    };
  }

  public static double minimumCycleStartRemainingSeconds(AutoSpec.RiskTier riskTier) {
    return switch (riskTier) {
      case SAFE -> 8.0;
      case BALANCED -> 6.0;
      case AGGRESSIVE -> 4.0;
    };
  }

  public static double parkReservationSeconds(AutoSpec.RiskTier riskTier) {
    return switch (riskTier) {
      case SAFE -> 5.0;
      case BALANCED -> 3.0;
      case AGGRESSIVE -> 1.5;
    };
  }

  public static boolean shouldStartCycle(AutoSpec spec, int cycleIndex, double elapsedSeconds) {
    if (cycleIndex >= effectiveCycleCount(spec)) {
      return false;
    }
    return remainingSeconds(elapsedSeconds) >= minimumCycleStartRemainingSeconds(spec.riskTier());
  }

  public static boolean shouldAttemptPark(AutoSpec spec, double elapsedSeconds) {
    if (spec.parkOption() == AutoSpec.ParkOption.NONE) {
      return false;
    }
    return remainingSeconds(elapsedSeconds) >= parkReservationSeconds(spec.riskTier());
  }

  public static double remainingSeconds(double elapsedSeconds) {
    return Math.max(0.0, AUTO_PERIOD_SECONDS - elapsedSeconds);
  }

  private static int cycleCap(AutoSpec.RiskTier riskTier) {
    return switch (riskTier) {
      case SAFE -> 1;
      case BALANCED, AGGRESSIVE -> 2;
    };
  }
}
