package frc.robot.superstructure.gamepiece;

public final class ShooterFeedInterlock {
  private ShooterFeedInterlock() {}

  public static boolean shouldAdvanceToShooter(
      boolean feedModeRequested, boolean shooterReadyToFire, boolean hasStagedGamePiece) {
    return feedModeRequested && shooterReadyToFire && hasStagedGamePiece;
  }

  public static boolean shouldRunIndexerDuringManualFeed(
      boolean manualFeedRequested, boolean autoAimActive, boolean shotSolutionFeasible) {
    return manualFeedRequested;
  }
}
