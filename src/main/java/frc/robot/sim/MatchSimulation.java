package frc.robot.sim;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.FieldTargets;
import frc.robot.MatchStateProvider;
import frc.robot.subsystems.endgame.SimpleEndgame;
import frc.robot.subsystems.indexer.IndexerGoal;
import frc.robot.subsystems.indexer.SimpleIndexer;
import frc.robot.subsystems.intake.SimpleIntake;
import frc.robot.subsystems.shooter.ShooterGoal;
import frc.robot.subsystems.shooter.ShotSolution;
import frc.robot.subsystems.shooter.SimpleShooter;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureDrive;
import frc.robot.subsystems.superstructure.SuperstructureGoal;
import frc.robot.subsystems.superstructure.SuperstructureStatus;
import java.util.Comparator;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

public class MatchSimulation {
  private static final double INTAKE_CAPTURE_RADIUS_METERS = 0.45;
  private static final double OUTPOST_REFILL_DISTANCE_METERS = 0.7;
  private static final double OUTPOST_REFILL_SECONDS = 0.35;
  private static final Translation2d INTAKE_OFFSET_METERS = new Translation2d(0.42, 0.0);
  private static final Translation2d SHOOTER_OFFSET_METERS = new Translation2d(0.08, 0.0);
  private static final Translation2d MAGAZINE_OFFSET_METERS = new Translation2d(0.05, 0.0);
  private static final Translation2d ENDGAME_LEFT_OFFSET_METERS = new Translation2d(-0.18, 0.22);
  private static final Translation2d ENDGAME_RIGHT_OFFSET_METERS = new Translation2d(-0.18, -0.22);
  private static final double INTAKE_HEIGHT_METERS = 0.18;
  private static final double SHOOTER_HEIGHT_METERS = 0.56;
  private static final double MAGAZINE_HEIGHT_METERS = 0.35;
  private static final double ENDGAME_BASE_HEIGHT_METERS = 0.78;
  private static final double EJECT_SPEED_METERS_PER_SEC = 1.8;

  private final SpartobotsArena2026Rebuilt arena;
  private final SuperstructureDrive drive;
  private final Superstructure superstructure;
  private final SimpleIntake intake;
  private final SimpleIndexer indexer;
  private final SimpleShooter shooter;
  private final SimpleEndgame endgame;

  private boolean previousHasGamePiece = false;
  private int shotAttempts = 0;
  private double outpostRefillTimerSeconds = 0.0;

  public MatchSimulation(
      SpartobotsArena2026Rebuilt arena,
      SuperstructureDrive drive,
      Superstructure superstructure,
      SimpleIntake intake,
      SimpleIndexer indexer,
      SimpleShooter shooter,
      SimpleEndgame endgame) {
    this.arena = arena;
    this.drive = drive;
    this.superstructure = superstructure;
    this.intake = intake;
    this.indexer = indexer;
    this.shooter = shooter;
    this.endgame = endgame;
  }

  public void reset(boolean preloadRobot) {
    indexer.setHoldingPiece(preloadRobot);
    previousHasGamePiece = preloadRobot;
    shotAttempts = 0;
    outpostRefillTimerSeconds = 0.0;
  }

  public void beforeArenaStep(
      MatchStateProvider.MatchState matchState, double matchTimeSeconds, Pose2d robotPose) {
    arena.updateMatchContext(matchState, matchTimeSeconds);

    SuperstructureStatus status = superstructure.getStatus();
    if (previousHasGamePiece && !status.hasGamePiece()) {
      handlePieceRelease(status, matchState);
    }

    tryAcquireFieldFuel(status, robotPose);
    tryRefillFromOutpost(status, matchState, robotPose);
    previousHasGamePiece = superstructure.getStatus().hasGamePiece();
  }

  public void logOutputs(MatchStateProvider.MatchState matchState, Pose3d robotPose3d) {
    Alliance alliance = matchState.alliance().orElse(Alliance.Blue);
    Alliance opponent = alliance == Alliance.Red ? Alliance.Blue : Alliance.Red;
    SuperstructureStatus status = superstructure.getStatus();

    Logger.recordOutput(
        "FieldSimulation/RobotParts/Intake",
        new Pose3d[] {
          poseFromRobot(
              robotPose3d, INTAKE_OFFSET_METERS, INTAKE_HEIGHT_METERS, getIntakePitchRadians())
        });
    Logger.recordOutput(
        "FieldSimulation/RobotParts/Shooter",
        new Pose3d[] {
          poseFromRobot(
              robotPose3d,
              SHOOTER_OFFSET_METERS,
              SHOOTER_HEIGHT_METERS,
              getShooterPitchRadians(status))
        });
    Logger.recordOutput(
        "FieldSimulation/RobotParts/Endgame",
        new Pose3d[] {
          poseFromRobot(
              robotPose3d,
              ENDGAME_LEFT_OFFSET_METERS,
              ENDGAME_BASE_HEIGHT_METERS + getEndgameExtensionMeters(),
              0.0),
          poseFromRobot(
              robotPose3d,
              ENDGAME_RIGHT_OFFSET_METERS,
              ENDGAME_BASE_HEIGHT_METERS + getEndgameExtensionMeters(),
              0.0)
        });
    Logger.recordOutput(
        "FieldSimulation/RobotParts/HeldFuel",
        status.hasGamePiece()
            ? new Pose3d[] {
              poseFromRobot(robotPose3d, MAGAZINE_OFFSET_METERS, MAGAZINE_HEIGHT_METERS, 0.0)
            }
            : new Pose3d[] {});

    Logger.recordOutput("FieldSimulation/Inventory/RobotFuelCount", status.hasGamePiece() ? 1 : 0);
    Logger.recordOutput(
        "FieldSimulation/Inventory/AllianceOutpostFuel", arena.getOutpostFuelCount(alliance));
    Logger.recordOutput(
        "FieldSimulation/Inventory/OpponentOutpostFuel", arena.getOutpostFuelCount(opponent));
    Logger.recordOutput(
        "FieldSimulation/Scoring/EffectiveActiveHub",
        SimMatchStateUtil.getEffectiveActiveHub(matchState).name());
    Logger.recordOutput("FieldSimulation/Scoring/AllianceScore", arena.getScore(alliance));
    Logger.recordOutput("FieldSimulation/Scoring/OpponentScore", arena.getScore(opponent));
    Logger.recordOutput(
        "FieldSimulation/Scoring/AllianceFuelInHub",
        arena.getBreakdownValue(alliance, "TotalFuelInHub"));
    Logger.recordOutput(
        "FieldSimulation/Scoring/OpponentFuelInHub",
        arena.getBreakdownValue(opponent, "TotalFuelInHub"));
    Logger.recordOutput(
        "FieldSimulation/Scoring/AllianceWastedFuel",
        arena.getBreakdownValue(alliance, "WastedFuel"));
    Logger.recordOutput(
        "FieldSimulation/Scoring/OpponentWastedFuel",
        arena.getBreakdownValue(opponent, "WastedFuel"));
    Logger.recordOutput(
        "FieldSimulation/Scoring/AllianceOutpostTotal",
        arena.getBreakdownValue(alliance, "TotalFuelInOutpost"));
    Logger.recordOutput(
        "FieldSimulation/Scoring/OpponentOutpostTotal",
        arena.getBreakdownValue(opponent, "TotalFuelInOutpost"));
    Logger.recordOutput("FieldSimulation/Scoring/ShotAttempts", shotAttempts);
  }

  private void handlePieceRelease(
      SuperstructureStatus status, MatchStateProvider.MatchState matchState) {
    indexer.setHoldingPiece(false);

    if (status.activeGoal() instanceof SuperstructureGoal.HubShot
        && shooter.getGoal() instanceof ShooterGoal.Fire
        && status.shotSolution() != null) {
      launchHubShot(status.shotSolution(), matchState);
      return;
    }

    if (status.activeGoal() instanceof SuperstructureGoal.Eject) {
      ejectFuel();
    }
  }

  private void launchHubShot(ShotSolution shotSolution, MatchStateProvider.MatchState matchState) {
    Alliance alliance = matchState.alliance().orElse(Alliance.Blue);
    RebuiltFuelOnFly projectile =
        new RebuiltFuelOnFly(
            drive.getPose().getTranslation(),
            SHOOTER_OFFSET_METERS,
            drive.getChassisSpeeds(),
            drive.getPose().getRotation(),
            Meters.of(shotSolution.launchHeightMeters()),
            MetersPerSecond.of(shotSolution.launchSpeedMetersPerSec()),
            Degrees.of(shotSolution.launchAngle().getDegrees()));
    projectile.withTargetPosition(
        () ->
            SimMatchStateUtil.getHubPose(shotSolution.selectedHub(), alliance, matchState)
                .getTranslation());
    arena.addGamePieceProjectile(projectile);
    shotAttempts++;
  }

  private void ejectFuel() {
    Pose2d robotPose = drive.getPose();
    Translation2d ejectPosition =
        robotPose.getTranslation().plus(INTAKE_OFFSET_METERS.rotateBy(robotPose.getRotation()));
    RebuiltFuelOnField fuel = new RebuiltFuelOnField(ejectPosition);
    fuel.setVelocity(
        new ChassisSpeeds(
            -EJECT_SPEED_METERS_PER_SEC * robotPose.getRotation().getCos(),
            -EJECT_SPEED_METERS_PER_SEC * robotPose.getRotation().getSin(),
            0.0));
    arena.addGamePiece(fuel);
  }

  private void tryAcquireFieldFuel(SuperstructureStatus status, Pose2d robotPose) {
    if (indexer.getStatus().holdingPiece()
        || !isCollecting()
        || (status.hasGamePiece() && status.pieceState() != Superstructure.PieceState.ACQUIRING)) {
      return;
    }

    Translation2d intakePosition =
        robotPose.getTranslation().plus(INTAKE_OFFSET_METERS.rotateBy(robotPose.getRotation()));
    arena.gamePiecesOnField().stream()
        .filter(piece -> "Fuel".equals(piece.getType()))
        .min(
            Comparator.comparingDouble(
                piece -> piece.getPoseOnField().getTranslation().getDistance(intakePosition)))
        .filter(
            piece ->
                piece.getPoseOnField().getTranslation().getDistance(intakePosition)
                    <= INTAKE_CAPTURE_RADIUS_METERS)
        .ifPresent(
            piece -> {
              arena.removeGamePiece(piece);
              indexer.setHoldingPiece(true);
            });
  }

  private void tryRefillFromOutpost(
      SuperstructureStatus status, MatchStateProvider.MatchState matchState, Pose2d robotPose) {
    if (!(status.activeGoal() instanceof SuperstructureGoal.OutpostAlign)
        || status.hasGamePiece()
        || indexer.getStatus().holdingPiece()
        || !status.driveAligned()) {
      outpostRefillTimerSeconds = 0.0;
      return;
    }

    Alliance alliance = matchState.alliance().orElse(Alliance.Blue);
    Pose2d outpostPose = FieldTargets.OUTPOST.forAlliance(alliance);
    if (robotPose.getTranslation().getDistance(outpostPose.getTranslation())
        > OUTPOST_REFILL_DISTANCE_METERS) {
      outpostRefillTimerSeconds = 0.0;
      return;
    }

    outpostRefillTimerSeconds += TimedRobot.kDefaultPeriod;
    if (outpostRefillTimerSeconds >= OUTPOST_REFILL_SECONDS && arena.consumeOutpostFuel(alliance)) {
      indexer.setHoldingPiece(true);
      outpostRefillTimerSeconds = 0.0;
    }
  }

  private boolean isCollecting() {
    return switch (intake.getGoal()) {
      case COLLECT_DEPOT, COLLECT_FLOOR -> indexer.getGoal() == IndexerGoal.RECEIVE;
      default -> false;
    };
  }

  private double getIntakePitchRadians() {
    return switch (intake.getGoal()) {
      case DEPLOY_DEPOT, DEPLOY_FLOOR, COLLECT_DEPOT, COLLECT_FLOOR, REVERSE -> Rotation2d
          .fromDegrees(-58.0)
          .getRadians();
      default -> 0.0;
    };
  }

  private double getShooterPitchRadians(SuperstructureStatus status) {
    if (status.shotSolution() != null) {
      return -status.shotSolution().launchAngle().getRadians();
    }

    return shooter.getGoal() instanceof ShooterGoal.Safe
        ? Rotation2d.fromDegrees(-35.0).getRadians()
        : 0.0;
  }

  private double getEndgameExtensionMeters() {
    return switch (endgame.getGoal()) {
      case PREPARE -> 0.18;
      case CONTACT -> 0.42;
      case LEVEL -> 0.65;
      case STOWED -> 0.0;
    };
  }

  private static Pose3d poseFromRobot(
      Pose3d robotPose3d, Translation2d xyOffset, double zOffset, double pitchRadians) {
    Rotation2d robotYaw = Rotation2d.fromRadians(robotPose3d.getRotation().getZ());
    Translation2d rotatedOffset = xyOffset.rotateBy(robotYaw);
    return new Pose3d(
        new Translation3d(
            robotPose3d.getX() + rotatedOffset.getX(),
            robotPose3d.getY() + rotatedOffset.getY(),
            robotPose3d.getZ() + zOffset),
        new Rotation3d(robotPose3d.getRotation().getX(), pitchRadians, robotYaw.getRadians()));
  }
}
