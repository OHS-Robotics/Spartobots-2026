package frc.robot.superstructure.gamepiece;

public final class ShooterFeedInterlock {
  private ShooterFeedInterlock() {}

  public static boolean shouldAdvanceToShooter(
      boolean feedModeRequested, boolean shotWindowAvailable, boolean hasStagedGamePiece) {
    return feedModeRequested && shotWindowAvailable && hasStagedGamePiece;
  }

  public static boolean shouldRunIndexerDuringManualFeed(boolean manualFeedRequested) {
    return manualFeedRequested;
  }
}
