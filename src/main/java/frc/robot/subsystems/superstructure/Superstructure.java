package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.TargetSelector;
import frc.robot.subsystems.endgame.Endgame;
import frc.robot.subsystems.endgame.EndgameGoal;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerGoal;
import frc.robot.subsystems.indexer.IndexerStatus;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeGoal;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterBallistics;
import frc.robot.subsystems.shooter.ShooterGoal;
import frc.robot.subsystems.shooter.ShooterStatus;
import frc.robot.subsystems.shooter.ShotSolution;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  private final SuperstructureDrive drive;
  private final Intake intake;
  private final Indexer indexer;
  private final Shooter shooter;
  private final Endgame endgame;
  private final ShooterBallistics shooterBallistics;
  private final Targeting targeting;

  private SuperstructureGoal requestedGoal = null;
  private SuperstructureGoal activeGoal = new SuperstructureGoal.Stow();
  private PieceState pieceState = PieceState.EMPTY;

  private ShotSolution activeShotSolution = null;
  private Pose2d activeTargetPose = Pose2d.kZero;
  private Rotation2d activeTargetHeading = Rotation2d.kZero;
  private boolean shooterReady = false;
  private boolean driveAligned = false;

  public Superstructure(
      SuperstructureDrive drive,
      Intake intake,
      Indexer indexer,
      Shooter shooter,
      Endgame endgame,
      ShooterBallistics shooterBallistics) {
    this(
        drive,
        intake,
        indexer,
        shooter,
        endgame,
        shooterBallistics,
        new Targeting() {
          @Override
          public TargetSelector.HubSelection getSelectedHub() {
            return TargetSelector.HubSelection.ACTIVE;
          }

          @Override
          public Pose3d getHubPose(TargetSelector.HubSelection selection) {
            return TargetSelector.getHubPose3d(selection);
          }

          @Override
          public Pose2d getAllianceOutpostPose() {
            return TargetSelector.getOutpostPose(TargetSelector.OutpostSelection.ALLIANCE);
          }
        });
  }

  public Superstructure(
      SuperstructureDrive drive,
      Intake intake,
      Indexer indexer,
      Shooter shooter,
      Endgame endgame,
      ShooterBallistics shooterBallistics,
      Targeting targeting) {
    this.drive = drive;
    this.intake = intake;
    this.indexer = indexer;
    this.shooter = shooter;
    this.endgame = endgame;
    this.shooterBallistics = shooterBallistics;
    this.targeting = targeting;
  }

  public void setGoal(SuperstructureGoal goal) {
    requestedGoal = goal;
  }

  public void clearGoal() {
    requestedGoal = null;
  }

  public SuperstructureStatus getStatus() {
    return new SuperstructureStatus(
        Optional.ofNullable(requestedGoal),
        activeGoal,
        pieceState,
        hasGamePiece(),
        shooterReady,
        driveAligned,
        activeTargetPose,
        activeTargetHeading,
        activeShotSolution,
        atGoal());
  }

  public boolean hasGamePiece() {
    return switch (pieceState) {
      case EMPTY, EJECTING -> false;
      case ACQUIRING, HELD, SHOOT_PREPPING, READY_TO_FIRE, FIRING -> true;
    };
  }

  public boolean atGoal() {
    boolean childGoalsSatisfied =
        intake.atGoal() && indexer.atGoal() && shooter.atGoal() && endgame.atGoal();

    if (!childGoalsSatisfied) {
      return false;
    }

    if (activeGoal instanceof SuperstructureGoal.HubShot hubShot) {
      return switch (hubShot.phase()) {
        case PREP -> true;
        case AIM -> shooterReady && driveAligned;
        case FIRE -> !hasGamePiece();
      };
    }

    if (activeGoal instanceof SuperstructureGoal.OutpostAlign) {
      return driveAligned;
    }

    if (activeGoal instanceof SuperstructureGoal.IntakeDepot intakeDepot) {
      return intakeDepot.phase() != SuperstructureGoal.IntakePhase.SETTLE
          || pieceState == PieceState.HELD;
    }

    if (activeGoal instanceof SuperstructureGoal.IntakeFloor intakeFloor) {
      return intakeFloor.phase() != SuperstructureGoal.IntakePhase.SETTLE
          || pieceState == PieceState.HELD;
    }

    if (activeGoal instanceof SuperstructureGoal.Eject eject) {
      return eject.phase() != SuperstructureGoal.EjectPhase.FIRE || !hasGamePiece();
    }

    return true;
  }

  public Command teleopHubShotCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return Commands.parallel(
        Commands.startEnd(
            () -> setGoal(new SuperstructureGoal.HubShot(SuperstructureGoal.HubShotPhase.AIM)),
            this::clearGoal,
            this),
        drive.faceTarget(xSupplier, ySupplier, this::getActiveTargetHeading));
  }

  public Command teleopOutpostAlignCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return Commands.parallel(
        Commands.startEnd(
            () -> setGoal(new SuperstructureGoal.OutpostAlign()), this::clearGoal, this),
        drive.faceTarget(xSupplier, ySupplier, this::getActiveTargetHeading));
  }

  @Override
  public void periodic() {
    SuperstructureGoal desiredGoal =
        requestedGoal == null ? new SuperstructureGoal.Stow() : requestedGoal;
    activeGoal = resolveActiveGoal(desiredGoal);

    updateTargeting(activeGoal);
    applyChildGoals(activeGoal);
    updateReadiness();
    updatePieceState(activeGoal);
    logState();
  }

  private SuperstructureGoal resolveActiveGoal(SuperstructureGoal desiredGoal) {
    if (desiredGoal instanceof SuperstructureGoal.Stow) {
      return new SuperstructureGoal.Stow();
    }

    if (desiredGoal instanceof SuperstructureGoal.OutpostAlign) {
      return new SuperstructureGoal.OutpostAlign();
    }

    if (desiredGoal instanceof SuperstructureGoal.IntakeDepot intakeDepot) {
      SuperstructureGoal.IntakePhase currentPhase =
          activeGoal instanceof SuperstructureGoal.IntakeDepot current
              ? current.phase()
              : SuperstructureGoal.IntakePhase.PREP;
      if (!(activeGoal instanceof SuperstructureGoal.IntakeDepot)
          || intakeDepot.phase().ordinal() < currentPhase.ordinal()) {
        currentPhase = SuperstructureGoal.IntakePhase.PREP;
      }
      if (currentPhase == SuperstructureGoal.IntakePhase.PREP
          && intakeDepot.phase().ordinal() > currentPhase.ordinal()
          && intake.atGoal()
          && indexer.atGoal()) {
        currentPhase = SuperstructureGoal.IntakePhase.CAPTURE;
      }
      if (currentPhase == SuperstructureGoal.IntakePhase.CAPTURE
          && intakeDepot.phase().ordinal() > currentPhase.ordinal()
          && captureHasSettled()) {
        currentPhase = SuperstructureGoal.IntakePhase.SETTLE;
      }
      return new SuperstructureGoal.IntakeDepot(currentPhase);
    }

    if (desiredGoal instanceof SuperstructureGoal.IntakeFloor intakeFloor) {
      SuperstructureGoal.IntakePhase currentPhase =
          activeGoal instanceof SuperstructureGoal.IntakeFloor current
              ? current.phase()
              : SuperstructureGoal.IntakePhase.PREP;
      if (!(activeGoal instanceof SuperstructureGoal.IntakeFloor)
          || intakeFloor.phase().ordinal() < currentPhase.ordinal()) {
        currentPhase = SuperstructureGoal.IntakePhase.PREP;
      }
      if (currentPhase == SuperstructureGoal.IntakePhase.PREP
          && intakeFloor.phase().ordinal() > currentPhase.ordinal()
          && intake.atGoal()
          && indexer.atGoal()) {
        currentPhase = SuperstructureGoal.IntakePhase.CAPTURE;
      }
      if (currentPhase == SuperstructureGoal.IntakePhase.CAPTURE
          && intakeFloor.phase().ordinal() > currentPhase.ordinal()
          && captureHasSettled()) {
        currentPhase = SuperstructureGoal.IntakePhase.SETTLE;
      }
      return new SuperstructureGoal.IntakeFloor(currentPhase);
    }

    if (desiredGoal instanceof SuperstructureGoal.HubShot hubShot) {
      SuperstructureGoal.HubShotPhase currentPhase =
          activeGoal instanceof SuperstructureGoal.HubShot current
              ? current.phase()
              : SuperstructureGoal.HubShotPhase.PREP;
      if (!(activeGoal instanceof SuperstructureGoal.HubShot)
          || hubShot.phase().ordinal() < currentPhase.ordinal()) {
        currentPhase = SuperstructureGoal.HubShotPhase.PREP;
      }
      if (currentPhase == SuperstructureGoal.HubShotPhase.PREP
          && hubShot.phase().ordinal() > currentPhase.ordinal()
          && shooter.atGoal()) {
        currentPhase = SuperstructureGoal.HubShotPhase.AIM;
      }
      if (currentPhase == SuperstructureGoal.HubShotPhase.AIM
          && hubShot.phase().ordinal() > currentPhase.ordinal()
          && shooterReady
          && driveAligned
          && hasGamePiece()) {
        currentPhase = SuperstructureGoal.HubShotPhase.FIRE;
      }
      return new SuperstructureGoal.HubShot(currentPhase);
    }

    if (desiredGoal instanceof SuperstructureGoal.Eject eject) {
      SuperstructureGoal.EjectPhase currentPhase =
          activeGoal instanceof SuperstructureGoal.Eject current
              ? current.phase()
              : SuperstructureGoal.EjectPhase.PREP;
      if (!(activeGoal instanceof SuperstructureGoal.Eject)
          || eject.phase().ordinal() < currentPhase.ordinal()) {
        currentPhase = SuperstructureGoal.EjectPhase.PREP;
      }
      if (currentPhase == SuperstructureGoal.EjectPhase.PREP
          && eject.phase().ordinal() > currentPhase.ordinal()
          && intake.atGoal()
          && indexer.atGoal()) {
        currentPhase = SuperstructureGoal.EjectPhase.FIRE;
      }
      return new SuperstructureGoal.Eject(currentPhase);
    }

    if (desiredGoal instanceof SuperstructureGoal.Endgame endgameGoal) {
      SuperstructureGoal.EndgamePhase currentPhase =
          activeGoal instanceof SuperstructureGoal.Endgame current
              ? current.phase()
              : SuperstructureGoal.EndgamePhase.PREP;
      if (!(activeGoal instanceof SuperstructureGoal.Endgame)
          || endgameGoal.phase().ordinal() < currentPhase.ordinal()) {
        currentPhase = SuperstructureGoal.EndgamePhase.PREP;
      }
      if (currentPhase == SuperstructureGoal.EndgamePhase.PREP
          && endgameGoal.phase().ordinal() > currentPhase.ordinal()
          && endgame.atGoal()) {
        currentPhase = SuperstructureGoal.EndgamePhase.CONTACT;
      }
      if (currentPhase == SuperstructureGoal.EndgamePhase.CONTACT
          && endgameGoal.phase().ordinal() > currentPhase.ordinal()
          && endgame.atGoal()) {
        currentPhase = SuperstructureGoal.EndgamePhase.LEVEL;
      }
      return new SuperstructureGoal.Endgame(currentPhase, endgameGoal.zone());
    }

    return new SuperstructureGoal.Stow();
  }

  private boolean captureHasSettled() {
    IndexerStatus indexerStatus = indexer.getStatus();
    return indexerStatus.holdingPiece()
        || pieceState == PieceState.HELD
        || pieceState == PieceState.SHOOT_PREPPING
        || pieceState == PieceState.READY_TO_FIRE
        || pieceState == PieceState.FIRING;
  }

  private void updateTargeting(SuperstructureGoal goal) {
    activeShotSolution = null;
    activeTargetPose = Pose2d.kZero;
    activeTargetHeading = Rotation2d.kZero;

    if (goal instanceof SuperstructureGoal.HubShot) {
      activeShotSolution = buildHubShotSolution();
      activeTargetPose = activeShotSolution.targetPose();
      activeTargetHeading = activeShotSolution.targetHeading();
      return;
    }

    if (goal instanceof SuperstructureGoal.OutpostAlign) {
      activeTargetPose = targeting.getAllianceOutpostPose();
      activeTargetHeading = activeTargetPose.getRotation();
    }
  }

  private void applyChildGoals(SuperstructureGoal goal) {
    if (goal instanceof SuperstructureGoal.Stow
        || goal instanceof SuperstructureGoal.OutpostAlign) {
      intake.setGoal(IntakeGoal.STOW);
      indexer.setGoal(IndexerGoal.HOLD);
      shooter.setGoal(new ShooterGoal.Stow());
      endgame.setGoal(EndgameGoal.STOWED);
      return;
    }

    if (goal instanceof SuperstructureGoal.IntakeDepot intakeDepot) {
      shooter.setGoal(new ShooterGoal.Stow());
      endgame.setGoal(EndgameGoal.STOWED);
      switch (intakeDepot.phase()) {
        case PREP:
          intake.setGoal(IntakeGoal.DEPLOY_DEPOT);
          indexer.setGoal(IndexerGoal.RECEIVE);
          break;
        case CAPTURE:
          intake.setGoal(IntakeGoal.COLLECT_DEPOT);
          indexer.setGoal(IndexerGoal.RECEIVE);
          break;
        case SETTLE:
          intake.setGoal(IntakeGoal.HOLD);
          indexer.setGoal(IndexerGoal.SETTLE);
          break;
      }
      return;
    }

    if (goal instanceof SuperstructureGoal.IntakeFloor intakeFloor) {
      shooter.setGoal(new ShooterGoal.Stow());
      endgame.setGoal(EndgameGoal.STOWED);
      switch (intakeFloor.phase()) {
        case PREP:
          intake.setGoal(IntakeGoal.DEPLOY_FLOOR);
          indexer.setGoal(IndexerGoal.RECEIVE);
          break;
        case CAPTURE:
          intake.setGoal(IntakeGoal.COLLECT_FLOOR);
          indexer.setGoal(IndexerGoal.RECEIVE);
          break;
        case SETTLE:
          intake.setGoal(IntakeGoal.HOLD);
          indexer.setGoal(IndexerGoal.SETTLE);
          break;
      }
      return;
    }

    if (goal instanceof SuperstructureGoal.HubShot hubShot) {
      intake.setGoal(IntakeGoal.STOW);
      endgame.setGoal(EndgameGoal.STOWED);
      if (activeShotSolution == null) {
        activeShotSolution = buildHubShotSolution();
        activeTargetPose = activeShotSolution.targetPose();
        activeTargetHeading = activeShotSolution.targetHeading();
      }
      switch (hubShot.phase()) {
        case PREP:
          indexer.setGoal(IndexerGoal.HOLD);
          shooter.setGoal(new ShooterGoal.Track(activeShotSolution));
          break;
        case AIM:
          indexer.setGoal(IndexerGoal.HOLD);
          shooter.setGoal(new ShooterGoal.Ready(activeShotSolution));
          break;
        case FIRE:
          intake.setGoal(IntakeGoal.HOLD);
          indexer.setGoal(IndexerGoal.FEED_SHOOTER);
          shooter.setGoal(new ShooterGoal.Fire(activeShotSolution));
          break;
      }
      return;
    }

    if (goal instanceof SuperstructureGoal.Eject eject) {
      shooter.setGoal(new ShooterGoal.Safe());
      endgame.setGoal(EndgameGoal.STOWED);
      switch (eject.phase()) {
        case PREP:
          intake.setGoal(IntakeGoal.HOLD);
          indexer.setGoal(IndexerGoal.HOLD);
          break;
        case FIRE:
          intake.setGoal(IntakeGoal.REVERSE);
          indexer.setGoal(IndexerGoal.PURGE);
          break;
      }
      return;
    }

    if (goal instanceof SuperstructureGoal.Endgame endgameGoal) {
      intake.setGoal(IntakeGoal.STOW);
      indexer.setGoal(IndexerGoal.HOLD);
      shooter.setGoal(new ShooterGoal.Safe());
      switch (endgameGoal.phase()) {
        case PREP:
          endgame.setGoal(EndgameGoal.PREPARE);
          break;
        case CONTACT:
          endgame.setGoal(EndgameGoal.CONTACT);
          break;
        case LEVEL:
          endgame.setGoal(EndgameGoal.LEVEL);
          break;
      }
    }
  }

  private void updateReadiness() {
    ShooterStatus shooterStatus = shooter.getStatus();
    shooterReady = shooterStatus.readyToShoot();

    if (activeGoal instanceof SuperstructureGoal.HubShot
        || activeGoal instanceof SuperstructureGoal.OutpostAlign) {
      driveAligned = drive.isPoseTrusted() && drive.isAimed(this::getActiveTargetHeading);
    } else {
      driveAligned = false;
    }
  }

  private void updatePieceState(SuperstructureGoal goal) {
    IndexerStatus indexerStatus = indexer.getStatus();
    if (goal instanceof SuperstructureGoal.Stow
        || goal instanceof SuperstructureGoal.OutpostAlign
        || goal instanceof SuperstructureGoal.Endgame) {
      if (indexerStatus.holdingPiece()) {
        pieceState = PieceState.HELD;
      }
      return;
    }

    if (goal instanceof SuperstructureGoal.IntakeDepot intakeDepot) {
      pieceState =
          switch (intakeDepot.phase()) {
            case PREP, CAPTURE -> PieceState.ACQUIRING;
            case SETTLE -> PieceState.HELD;
          };
      return;
    }

    if (goal instanceof SuperstructureGoal.IntakeFloor intakeFloor) {
      pieceState =
          switch (intakeFloor.phase()) {
            case PREP, CAPTURE -> PieceState.ACQUIRING;
            case SETTLE -> PieceState.HELD;
          };
      return;
    }

    if (goal instanceof SuperstructureGoal.HubShot hubShot) {
      if (!hasGamePiece() && pieceState != PieceState.FIRING) {
        return;
      }
      switch (hubShot.phase()) {
        case PREP:
          pieceState = PieceState.SHOOT_PREPPING;
          break;
        case AIM:
          pieceState = PieceState.READY_TO_FIRE;
          break;
        case FIRE:
          pieceState = indexer.getStatus().atGoal() ? PieceState.EMPTY : PieceState.FIRING;
          break;
      }
      return;
    }

    if (goal instanceof SuperstructureGoal.Eject eject) {
      if (eject.phase() == SuperstructureGoal.EjectPhase.FIRE) {
        pieceState = indexer.getStatus().atGoal() ? PieceState.EMPTY : PieceState.EJECTING;
      }
      return;
    }
  }

  private Rotation2d getActiveTargetHeading() {
    return activeTargetHeading;
  }

  private ShotSolution buildHubShotSolution() {
    TargetSelector.HubSelection selectedHub = targeting.getSelectedHub();
    return shooterBallistics.solveHubShot(
        drive.getPose(),
        drive.getChassisSpeeds(),
        selectedHub,
        pieceState,
        targeting.getHubPose(selectedHub));
  }

  private void logState() {
    Logger.recordOutput(
        "Superstructure/RequestedGoal", requestedGoal == null ? "NONE" : requestedGoal.toString());
    Logger.recordOutput("Superstructure/ActiveGoal", activeGoal.toString());
    Logger.recordOutput("Superstructure/PieceState", pieceState.name());
    Logger.recordOutput("Superstructure/HasGamePiece", hasGamePiece());
    Logger.recordOutput("Superstructure/ShooterReady", shooterReady);
    Logger.recordOutput("Superstructure/DriveAligned", driveAligned);
    Logger.recordOutput("Superstructure/PoseTrusted", drive.isPoseTrusted());
    Logger.recordOutput("Superstructure/AtGoal", atGoal());
    Logger.recordOutput("Superstructure/TargetPose", activeTargetPose);
    Logger.recordOutput(
        "Superstructure/TargetHeadingDegrees",
        Units.radiansToDegrees(activeTargetHeading.getRadians()));
    Logger.recordOutput("Superstructure/IntakeGoal", intake.getGoal().name());
    Logger.recordOutput("Superstructure/IndexerGoal", indexer.getGoal().name());
    Logger.recordOutput("Superstructure/EndgameGoal", endgame.getGoal().name());
    Logger.recordOutput("Superstructure/ShooterGoal", shooter.getGoal().toString());

    double launchSpeed = 0.0;
    double launchAngleDegrees = 0.0;
    double launchHeight = 0.0;
    double airtime = 0.0;
    String selectedHub = "NONE";
    String solutionPieceState = "NONE";
    if (activeShotSolution != null) {
      selectedHub = activeShotSolution.selectedHub().name();
      solutionPieceState = activeShotSolution.pieceState().name();
      launchSpeed = activeShotSolution.launchSpeedMetersPerSec();
      launchAngleDegrees = activeShotSolution.launchAngle().getDegrees();
      launchHeight = activeShotSolution.launchHeightMeters();
      airtime = activeShotSolution.airtimeSeconds();
    }
    Logger.recordOutput("Superstructure/Shot/SelectedHub", selectedHub);
    Logger.recordOutput("Superstructure/Shot/PieceState", solutionPieceState);
    Logger.recordOutput("Superstructure/Shot/LaunchSpeedMetersPerSec", launchSpeed);
    Logger.recordOutput("Superstructure/Shot/LaunchAngleDegrees", launchAngleDegrees);
    Logger.recordOutput("Superstructure/Shot/LaunchHeightMeters", launchHeight);
    Logger.recordOutput("Superstructure/Shot/AirtimeSeconds", airtime);
  }

  public static enum PieceState {
    EMPTY,
    ACQUIRING,
    HELD,
    SHOOT_PREPPING,
    READY_TO_FIRE,
    FIRING,
    EJECTING
  }

  public interface Targeting {
    public TargetSelector.HubSelection getSelectedHub();

    public Pose3d getHubPose(TargetSelector.HubSelection selection);

    public Pose2d getAllianceOutpostPose();
  }
}
