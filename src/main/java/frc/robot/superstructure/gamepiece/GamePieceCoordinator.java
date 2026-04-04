package frc.robot.superstructure.gamepiece;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.gamepiece.hopper.Hopper;
import frc.robot.subsystems.gamepiece.indexers.Indexers;
import frc.robot.subsystems.gamepiece.intake.Intake;
import frc.robot.subsystems.gamepiece.shooter.Shooter;
import frc.robot.util.NetworkTablesUtil;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class GamePieceCoordinator {
  private static final double SHOOTER_TRIGGER_DEADBAND = 0.02;
  private static final double MANUAL_FEED_TRIGGER_DEADBAND = 0.02;
  private static final double BELT_MANUAL_FEED_SPEED = 1.0;
  private static final double SHARED_INDEXER_BREAKAWAY_SPEED = 1.0;
  private static final double SHARED_INDEXER_HOLD_SPEED = 0.9;
  private static final double SHARED_INDEXER_BREAKAWAY_SECONDS = 0.40;
  private static final double BASIC_COLLECT_INTAKE_SPEED = 0.65;
  private static final double BASIC_COLLECT_BELT_SPEED = 0.55;
  private static final double BASIC_COLLECT_INDEXER_SPEED = 0.55;
  private static final double BASIC_FEED_BELT_SPEED = 0.75;
  private static final double BASIC_FEED_INDEXER_SPEED = 0.75;
  private static final double BASIC_REVERSE_SPEED = -0.45;
  private static final String logRoot = NetworkTablesUtil.logPath("Operator/GamePieceControl");

  private final Intake intake;
  private final Hopper hopper;
  private final Indexers indexers;
  private final Shooter shooter;

  private boolean shooterDemandFromAlign = false;
  private double shooterDemandFromTriggerThrottle = 0.0;
  private double sharedIndexerBreakawayUntilTimestampSeconds = 0.0;

  public GamePieceCoordinator(Intake intake, Hopper hopper, Indexers indexers, Shooter shooter) {
    this.intake = intake;
    this.hopper = hopper;
    this.indexers = indexers;
    this.shooter = shooter;
  }

  public Command runShooterDemandWhileHeldCommand(DoubleSupplier throttleSupplier) {
    return Commands.runEnd(
        () -> setShooterDemandFromTriggerThrottle(throttleSupplier.getAsDouble()),
        () -> setShooterDemandFromTriggerThrottle(0.0));
  }

  public Command runHopperExtensionWhileHeldCommand(double speed) {
    return Commands.runEnd(
        () -> hopper.setHopperExtensionSpeed(speed), hopper::stopHopperExtension, hopper);
  }

  public Command basicCollectWhileHeldCommand(boolean runIndexers) {
    return Commands.startEnd(
        () -> applyBasicCollect(runIndexers), this::stopGamePieceFlow, intake, indexers);
  }

  public Command basicReverseWhileHeldCommand() {
    return Commands.startEnd(this::applyBasicReverse, this::stopGamePieceFlow, intake, indexers);
  }

  public Command runManualFeedAndIndexersWhileHeldCommand(DoubleSupplier throttleSupplier) {
    return Commands.runEnd(
            () -> applyManualFeed(throttleSupplier.getAsDouble()),
            () -> {
              intake.stopIntake();
              indexers.stopIndexers();
            },
            intake,
            indexers)
        .beforeStarting(
            () ->
                sharedIndexerBreakawayUntilTimestampSeconds =
                    Timer.getFPGATimestamp() + SHARED_INDEXER_BREAKAWAY_SECONDS);
  }

  public void applyBasicCollect(boolean runIndexers) {
    intake.setTargetIntakeSpeed(BASIC_COLLECT_INTAKE_SPEED);
    intake.updateIntake();
    indexers.setTargetBottomIndexerSpeed(BASIC_COLLECT_BELT_SPEED);
    if (runIndexers) {
      indexers.setTargetTopIndexerSpeed(BASIC_COLLECT_INDEXER_SPEED);
      indexers.updateIndexers();
      recordMode("COLLECT");
    } else {
      indexers.setTargetTopIndexerSpeed(0.0);
      indexers.updateIndexers();
      recordMode("COLLECT_NO_INDEXER");
    }
  }

  public void applyBasicFeed(boolean runIndexers) {
    intake.stopIntake();
    if (runIndexers) {
      boolean feedAllowed =
          ShooterFeedInterlock.shouldAdvanceToShooter(
              true,
              shooter.isReadyToFire(),
              true /* Sensorless path: assume staged piece is present when feed is requested. */);
      if (feedAllowed) {
        indexers.setTargetTopIndexerSpeed(BASIC_FEED_INDEXER_SPEED);
        indexers.setTargetBottomIndexerSpeed(BASIC_FEED_BELT_SPEED);
        indexers.updateIndexers();
        recordMode("FEED");
      } else {
        indexers.stopIndexers();
        recordMode("FEED_INTERLOCKED");
      }
    } else {
      indexers.setTargetTopIndexerSpeed(0.0);
      indexers.setTargetBottomIndexerSpeed(BASIC_FEED_BELT_SPEED);
      indexers.updateIndexers();
      recordMode("MANUAL_FEED");
    }
  }

  public void applyBasicReverse() {
    intake.setTargetIntakeSpeed(BASIC_REVERSE_SPEED);
    indexers.setTargetIndexerSpeed(BASIC_REVERSE_SPEED);
    intake.updateIntake();
    indexers.updateIndexers();
    recordMode("REVERSE");
  }

  public void stopGamePieceFlow() {
    intake.stopIntake();
    indexers.stopIndexers();
    recordMode("IDLE");
  }

  public void setShooterDemandFromAlign(boolean enabled) {
    if (enabled) {
      shooter.setManualHoodOverrideEnabled(false);
    }
    shooterDemandFromAlign = enabled;
    refreshShooterControlDemand();
  }

  public void onDisabledInit() {
    shooterDemandFromAlign = false;
    shooterDemandFromTriggerThrottle = 0.0;
    refreshShooterControlDemand();
    stopGamePieceFlow();
  }

  public double getBasicFeedIndexerSpeed() {
    return BASIC_FEED_INDEXER_SPEED;
  }

  private void applyManualFeed(double throttle) {
    intake.stopIntake();
    boolean allowFeedPath =
        ShooterFeedInterlock.shouldRunIndexerDuringManualFeed(
            true, shooterDemandFromAlign, shooter.isHubShotSolutionFeasible());
    if (!allowFeedPath) {
      indexers.stopIndexers();
      recordMode("MANUAL_FEED_INTERLOCKED");
      Logger.recordOutput(logRoot + "/State/ManualFeedThrottleScale", 0.0);
      return;
    }

    boolean inSharedIndexerBreakaway =
        Timer.getFPGATimestamp() < sharedIndexerBreakawayUntilTimestampSeconds;
    double throttleScale = getManualFeedThrottleScale(throttle);
    double sharedIndexerOutput =
        -(inSharedIndexerBreakaway ? SHARED_INDEXER_BREAKAWAY_SPEED : SHARED_INDEXER_HOLD_SPEED)
            * throttleScale;
    double beltOutput = BELT_MANUAL_FEED_SPEED * throttleScale;
    indexers.setManualOutputs(sharedIndexerOutput, beltOutput);
    recordMode("MANUAL_FEED");
    Logger.recordOutput(logRoot + "/State/ManualFeedThrottleScale", throttleScale);
  }

  private double getManualFeedThrottleScale(double throttle) {
    double clampedThrottle = MathUtil.clamp(throttle, 0.0, 1.0);
    return MathUtil.applyDeadband(clampedThrottle, MANUAL_FEED_TRIGGER_DEADBAND);
  }

  private void setShooterDemandFromTriggerThrottle(double throttle) {
    shooterDemandFromTriggerThrottle = MathUtil.clamp(throttle, 0.0, 1.0);
    refreshShooterControlDemand();
  }

  private void refreshShooterControlDemand() {
    boolean shooterDemandFromTrigger = shooterDemandFromTriggerThrottle > SHOOTER_TRIGGER_DEADBAND;
    boolean shooterDemandEnabled = shooterDemandFromAlign || shooterDemandFromTrigger;
    double shooterThrottleScale =
        shooterDemandEnabled
            ? (shooterDemandFromAlign ? 1.0 : shooterDemandFromTriggerThrottle)
            : 1.0;
    shooter.setOperatorWheelThrottleScale(shooterThrottleScale);
    shooter.setShotControlEnabled(shooterDemandEnabled);
  }

  private void recordMode(String mode) {
    Logger.recordOutput(logRoot + "/State/Mode", mode);
  }
}
