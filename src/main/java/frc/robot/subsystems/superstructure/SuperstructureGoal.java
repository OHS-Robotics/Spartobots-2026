package frc.robot.subsystems.superstructure;

import frc.robot.TargetSelector;

public sealed interface SuperstructureGoal
    permits SuperstructureGoal.Stow,
        SuperstructureGoal.IntakeDepot,
        SuperstructureGoal.IntakeFloor,
        SuperstructureGoal.HubShot,
        SuperstructureGoal.OutpostAlign,
        SuperstructureGoal.Eject,
        SuperstructureGoal.Endgame {
  public record Stow() implements SuperstructureGoal {}

  public record IntakeDepot(IntakePhase phase) implements SuperstructureGoal {}

  public record IntakeFloor(IntakePhase phase) implements SuperstructureGoal {}

  public record HubShot(HubShotPhase phase) implements SuperstructureGoal {}

  public record OutpostAlign() implements SuperstructureGoal {}

  public record Eject(EjectPhase phase) implements SuperstructureGoal {}

  public record Endgame(EndgamePhase phase, TargetSelector.ParkZoneSelection zone)
      implements SuperstructureGoal {}

  public static enum IntakePhase {
    PREP,
    CAPTURE,
    SETTLE
  }

  public static enum HubShotPhase {
    PREP,
    AIM,
    FIRE
  }

  public static enum EjectPhase {
    PREP,
    FIRE
  }

  public static enum EndgamePhase {
    PREP,
    CONTACT,
    LEVEL
  }
}
