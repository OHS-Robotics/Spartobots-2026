package frc.robot.subsystems.body;

public final class ShooterFeedInterlock {
  private ShooterFeedInterlock() {}

  public static boolean shouldAdvanceToShooter(
      boolean feedModeRequested, boolean shooterReadyToFire, boolean hasStagedGamePiece) {
    return feedModeRequested && shooterReadyToFire && hasStagedGamePiece;
  }
}
