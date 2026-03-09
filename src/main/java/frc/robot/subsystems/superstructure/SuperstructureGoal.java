package frc.robot.subsystems.superstructure;

import frc.robot.RobotAction;
import frc.robot.TargetSelector;

public sealed interface SuperstructureGoal
    permits SuperstructureGoal.Stow,
        SuperstructureGoal.IntakeDepot,
        SuperstructureGoal.IntakeFloor,
        SuperstructureGoal.HubShot,
        SuperstructureGoal.OutpostAlign,
        SuperstructureGoal.Eject,
        SuperstructureGoal.Endgame {
  RobotAction action();

  public record Stow() implements SuperstructureGoal {
    @Override
    public RobotAction action() {
      return RobotAction.IDLE;
    }
  }

  public record IntakeDepot(IntakePhase phase) implements SuperstructureGoal {
    @Override
    public RobotAction action() {
      return RobotAction.ACQUIRE_DEPOT;
    }
  }

  public record IntakeFloor(IntakePhase phase) implements SuperstructureGoal {
    @Override
    public RobotAction action() {
      return RobotAction.ACQUIRE_FLOOR;
    }
  }

  public record HubShot(HubShotPhase phase) implements SuperstructureGoal {
    @Override
    public RobotAction action() {
      return RobotAction.AUTO_FACE_AND_SCORE;
    }
  }

  public record OutpostAlign() implements SuperstructureGoal {
    @Override
    public RobotAction action() {
      return RobotAction.OUTPOST_FEED;
    }
  }

  public record Eject(EjectPhase phase) implements SuperstructureGoal {
    @Override
    public RobotAction action() {
      return RobotAction.RECOVER;
    }
  }

  public record Endgame(EndgamePhase phase, TargetSelector.ParkZoneSelection zone)
      implements SuperstructureGoal {
    @Override
    public RobotAction action() {
      return RobotAction.QUICK_PARK;
    }
  }

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
