package frc.robot.subsystems.body;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.NetworkTablesUtil;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class GamePieceManager extends SubsystemBase {
  public enum Mode {
    IDLE,
    COLLECT,
    COLLECT_NO_INDEXER,
    HOLD,
    FEED,
    MANUAL_FEED,
    REVERSE,
    UNJAM
  }

  private final Intake intake;
  private final Hopper hopper;
  private final Indexers indexers;
  private final GamePieceSensorIO sensorIO;
  private final GamePieceSensorIOInputsAutoLogged sensorInputs =
      new GamePieceSensorIOInputsAutoLogged();

  private final NetworkTable subsystemTable =
      NetworkTablesUtil.subsystemTable(GamePieceManagerConstants.configTableName);
  private final NetworkTable tuningTable = NetworkTablesUtil.tuningCommonTable(subsystemTable);
  private final NetworkTableEntry useDashboardSensorOverridesEntry =
      tuningTable.getEntry("SensorOverrides/UseDashboardOverrides");
  private final NetworkTableEntry intakeDetectedOverrideEntry =
      tuningTable.getEntry("SensorOverrides/IntakeDetected");
  private final NetworkTableEntry hopperDetectedOverrideEntry =
      tuningTable.getEntry("SensorOverrides/HopperDetected");
  private final NetworkTableEntry shooterDetectedOverrideEntry =
      tuningTable.getEntry("SensorOverrides/ShooterDetected");

  private final Timer modeTimer = new Timer();
  private final Timer jamTimer = new Timer();
  private final Timer unjamTimer = new Timer();

  private Mode mode = Mode.IDLE;
  private Mode modeBeforeUnjam = Mode.IDLE;
  private BooleanSupplier shooterReadySupplier = () -> false;
  private BooleanSupplier autoAimActiveSupplier = () -> false;
  private BooleanSupplier shotSolutionFeasibleSupplier = () -> true;
  private boolean intakeDetected = false;
  private boolean hopperDetected = false;
  private boolean shooterDetected = false;
  private boolean jamTimerActive = false;

  public GamePieceManager(Intake intake, Hopper hopper, Indexers indexers, GamePieceSensorIO sensorIO) {
    this.intake = intake;
    this.hopper = hopper;
    this.indexers = indexers;
    this.sensorIO = sensorIO;

    useDashboardSensorOverridesEntry.setDefaultBoolean(false);
    intakeDetectedOverrideEntry.setDefaultBoolean(false);
    hopperDetectedOverrideEntry.setDefaultBoolean(false);
    shooterDetectedOverrideEntry.setDefaultBoolean(false);

    modeTimer.start();
  }

  @Override
  public void periodic() {
    sensorIO.updateInputs(sensorInputs);
    Logger.processInputs("GamePieceManager/Sensors", sensorInputs);
    updateSensors();
    updateJamDetection();
    runMode();
    logState();
  }

  public void setShooterReadySupplier(BooleanSupplier shooterReadySupplier) {
    this.shooterReadySupplier = shooterReadySupplier != null ? shooterReadySupplier : () -> false;
  }

  public void setManualFeedInterlockSuppliers(
      BooleanSupplier autoAimActiveSupplier, BooleanSupplier shotSolutionFeasibleSupplier) {
    this.autoAimActiveSupplier =
        autoAimActiveSupplier != null ? autoAimActiveSupplier : () -> false;
    this.shotSolutionFeasibleSupplier =
        shotSolutionFeasibleSupplier != null ? shotSolutionFeasibleSupplier : () -> true;
  }

  public Mode getMode() {
    return mode;
  }

  public void requestMode(Mode newMode) {
    if (newMode == mode) {
      return;
    }
    mode = newMode;
    modeTimer.restart();
    if (mode != Mode.UNJAM) {
      unjamTimer.stop();
      unjamTimer.reset();
    }
  }

  public Command collectWhileHeldCommand() {
    return Commands.startEnd(() -> requestMode(Mode.COLLECT), () -> requestMode(Mode.HOLD), this);
  }

  public Command collectWithoutIndexerWhileHeldCommand() {
    return Commands.startEnd(
        () -> requestMode(Mode.COLLECT_NO_INDEXER), () -> requestMode(Mode.HOLD), this);
  }

  public Command feedWhileHeldCommand() {
    return Commands.startEnd(() -> requestMode(Mode.FEED), () -> requestMode(Mode.HOLD), this);
  }

  public Command manualFeedWhileHeldCommand() {
    return Commands.startEnd(
        () -> requestMode(Mode.MANUAL_FEED), () -> requestMode(Mode.HOLD), this);
  }

  public Command reverseWhileHeldCommand() {
    return Commands.startEnd(() -> requestMode(Mode.REVERSE), () -> requestMode(Mode.IDLE), this);
  }

  public Command setModeCommand(Mode targetMode) {
    return Commands.runOnce(() -> requestMode(targetMode), this);
  }

  public boolean hasGamePieceAtIntake() {
    return intakeDetected;
  }

  public boolean hasGamePieceAtHopper() {
    return hopperDetected;
  }

  public boolean hasGamePieceAtShooter() {
    return shooterDetected;
  }

  public double getFeedRateRatioForShooterSim() {
    if (mode != Mode.FEED && mode != Mode.MANUAL_FEED) {
      return 0.0;
    }

    double nominalFeedIndexerOutput = Math.abs(GamePieceManagerConstants.feedIndexerSpeed);
    if (nominalFeedIndexerOutput < 1e-6) {
      return 0.0;
    }

    return MathUtil.clamp(
        Math.abs(indexers.getAverageAppliedOutput()) / nominalFeedIndexerOutput, 0.0, 1.0);
  }

  public boolean isManualFeedIndexerAllowedForSimulation() {
    return ShooterFeedInterlock.shouldRunIndexerDuringManualFeed(
        mode == Mode.MANUAL_FEED,
        autoAimActiveSupplier.getAsBoolean(),
        shotSolutionFeasibleSupplier.getAsBoolean());
  }

  private void runMode() {
    switch (mode) {
      case IDLE:
        applyIdle();
        break;
      case COLLECT:
        if (hasMeasuredStagedPiece()) {
          requestMode(Mode.HOLD);
          applyHold();
          break;
        }
        if (shouldAutoHoldDuringSensorlessCollect()) {
          requestMode(Mode.HOLD);
          applyHold();
          break;
        }
        applyCollect();
        break;
      case COLLECT_NO_INDEXER:
        if (hasMeasuredStagedPiece()) {
          requestMode(Mode.HOLD);
          applyHold();
          break;
        }
        if (shouldAutoHoldDuringSensorlessCollect()) {
          requestMode(Mode.HOLD);
          applyHold();
          break;
        }
        applyCollectWithoutIndexer();
        break;
      case HOLD:
        applyHold();
        break;
      case FEED:
        boolean feedAllowed =
            ShooterFeedInterlock.shouldAdvanceToShooter(
                true, shooterReadySupplier.getAsBoolean(), hasPieceAvailableForFeed());
        if (feedAllowed) {
          applyFeed();
        } else {
          applyHold();
        }
        break;
      case MANUAL_FEED:
        applyManualFeed(isManualFeedIndexerAllowedForSimulation());
        break;
      case REVERSE:
        applyReverse();
        break;
      case UNJAM:
        applyReverse();
        if (unjamTimer.hasElapsed(GamePieceManagerConstants.unjamReverseSeconds)) {
          requestMode(modeBeforeUnjam);
        }
        break;
    }
  }

  private void updateSensors() {
    boolean useOverrides = useDashboardSensorOverridesEntry.getBoolean(false);
    intakeDetected = readSensor(sensorInputs.intakeDetected, intakeDetectedOverrideEntry, useOverrides);
    hopperDetected = readSensor(sensorInputs.hopperDetected, hopperDetectedOverrideEntry, useOverrides);
    shooterDetected =
        readSensor(sensorInputs.shooterDetected, shooterDetectedOverrideEntry, useOverrides);
  }

  private void updateJamDetection() {
    if (mode == Mode.COLLECT
        || mode == Mode.COLLECT_NO_INDEXER
        || mode == Mode.FEED
        || mode == Mode.MANUAL_FEED) {
      if (getFeedCurrentAmps() >= GamePieceManagerConstants.jamCurrentThresholdAmps) {
        if (!jamTimerActive) {
          jamTimer.restart();
          jamTimerActive = true;
        }
      } else {
        jamTimer.stop();
        jamTimer.reset();
        jamTimerActive = false;
      }

      if (mode != Mode.UNJAM
          && jamTimer.hasElapsed(GamePieceManagerConstants.jamDetectionSeconds)) {
        modeBeforeUnjam = mode;
        mode = Mode.UNJAM;
        modeTimer.restart();
        unjamTimer.restart();
        jamTimer.stop();
        jamTimer.reset();
        jamTimerActive = false;
      }
    } else {
      jamTimer.stop();
      jamTimer.reset();
      jamTimerActive = false;
    }
  }

  private void applyIdle() {
    stopFeedMotors();
  }

  private void applyCollect() {
    intake.setTargetIntakeSpeed(GamePieceManagerConstants.collectIntakeSpeed);
    hopper.setTargetAgitatorSpeed(GamePieceManagerConstants.collectAgitatorSpeed);
    indexers.setTargetIndexerSpeed(GamePieceManagerConstants.collectIndexerSpeed);
    intake.updateIntake();
    hopper.updateAgitator();
    indexers.updateIndexers();
  }

  private void applyCollectWithoutIndexer() {
    intake.setTargetIntakeSpeed(GamePieceManagerConstants.collectIntakeSpeed);
    hopper.setTargetAgitatorSpeed(GamePieceManagerConstants.collectAgitatorSpeed);
    intake.updateIntake();
    hopper.updateAgitator();
    indexers.stopIndexers();
  }

  private void applyHold() {
    stopFeedMotors();
  }

  private void applyFeed() {
    intake.stopIntake();
    hopper.setTargetAgitatorSpeed(GamePieceManagerConstants.feedAgitatorSpeed);
    indexers.setTargetIndexerSpeed(GamePieceManagerConstants.feedIndexerSpeed);
    hopper.updateAgitator();
    indexers.updateIndexers();
  }

  private void applyManualFeed(boolean allowIndexer) {
    intake.stopIntake();
    hopper.setTargetAgitatorSpeed(GamePieceManagerConstants.feedAgitatorSpeed);
    hopper.updateAgitator();
    if (allowIndexer) {
      indexers.setTargetIndexerSpeed(GamePieceManagerConstants.feedIndexerSpeed);
      indexers.updateIndexers();
    } else {
      indexers.stopIndexers();
    }
  }

  private void applyReverse() {
    intake.setTargetIntakeSpeed(GamePieceManagerConstants.reverseSpeed);
    hopper.setTargetAgitatorSpeed(GamePieceManagerConstants.reverseSpeed);
    indexers.setTargetIndexerSpeed(GamePieceManagerConstants.reverseSpeed);
    intake.updateIntake();
    hopper.updateAgitator();
    indexers.updateIndexers();
  }

  private void stopFeedMotors() {
    intake.stopIntake();
    hopper.stopAgitator();
    indexers.stopIndexers();
  }

  private boolean hasMeasuredStagedPiece() {
    return shooterDetected || hopperDetected;
  }

  private boolean hasPieceAvailableForFeed() {
    return hasAnyBeamBreakConfigured() ? hasMeasuredStagedPiece() : true;
  }

  private boolean hasAnyBeamBreakConfigured() {
    return sensorInputs.intakeConfigured
        || sensorInputs.hopperConfigured
        || sensorInputs.shooterConfigured;
  }

  private boolean shouldAutoHoldDuringSensorlessCollect() {
    return !hasAnyBeamBreakConfigured()
        && GamePieceManagerConstants.sensorlessCollectToHoldSeconds > 0.0
        && modeTimer.hasElapsed(GamePieceManagerConstants.sensorlessCollectToHoldSeconds);
  }

  private double getFeedCurrentAmps() {
    return intake.getIntakeDriveCurrentAmps()
        + hopper.getHopperAgitatorCurrentAmps()
        + indexers.getAverageIndexerCurrentAmps();
  }

  private static boolean readSensor(
      boolean sensorDetected, NetworkTableEntry overrideEntry, boolean useOverrides) {
    return sensorDetected || (useOverrides && overrideEntry.getBoolean(false));
  }

  private void logState() {
    boolean autoAimActive = autoAimActiveSupplier.getAsBoolean();
    boolean shotSolutionFeasible = shotSolutionFeasibleSupplier.getAsBoolean();
    boolean manualFeedIndexerAllowed = isManualFeedIndexerAllowedForSimulation();
    Logger.recordOutput("GamePieceManager/Mode", mode.name());
    Logger.recordOutput("GamePieceManager/Sensor/IntakeDetected", intakeDetected);
    Logger.recordOutput("GamePieceManager/Sensor/HopperDetected", hopperDetected);
    Logger.recordOutput("GamePieceManager/Sensor/ShooterDetected", shooterDetected);
    Logger.recordOutput(
        "GamePieceManager/Interlock/ShooterReady", shooterReadySupplier.getAsBoolean());
    Logger.recordOutput(
        "GamePieceManager/Interlock/PieceAvailableForFeed", hasPieceAvailableForFeed());
    Logger.recordOutput("GamePieceManager/Interlock/AutoAimActive", autoAimActive);
    Logger.recordOutput("GamePieceManager/Interlock/ShotSolutionFeasible", shotSolutionFeasible);
    Logger.recordOutput(
        "GamePieceManager/Interlock/ManualFeedIndexerAllowed", manualFeedIndexerAllowed);
    Logger.recordOutput("GamePieceManager/Current/FeedCurrentAmps", getFeedCurrentAmps());
  }
}
