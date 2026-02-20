package frc.robot.subsystems.body;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class GamePieceManager extends SubsystemBase {
  public enum Mode {
    IDLE,
    COLLECT,
    HOLD,
    FEED,
    REVERSE,
    UNJAM
  }

  private final Intake intake;
  private final Hopper hopper;
  private final Agitators agitators;

  private final DigitalInput intakeBeamBreak;
  private final DigitalInput hopperBeamBreak;
  private final DigitalInput shooterBeamBreak;

  private final NetworkTable subsystemTable =
      NetworkTableInstance.getDefault().getTable(GamePieceManagerConstants.configTableName);
  private final NetworkTable tuningTable = subsystemTable.getSubTable("Tuning");
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
  private boolean intakeDetected = false;
  private boolean hopperDetected = false;
  private boolean shooterDetected = false;
  private boolean jamTimerActive = false;

  public GamePieceManager(Intake intake, Hopper hopper, Agitators agitators) {
    this.intake = intake;
    this.hopper = hopper;
    this.agitators = agitators;

    intakeBeamBreak = createBeamBreak(GamePieceManagerConstants.intakeBeamBreakChannel);
    hopperBeamBreak = createBeamBreak(GamePieceManagerConstants.hopperBeamBreakChannel);
    shooterBeamBreak = createBeamBreak(GamePieceManagerConstants.shooterBeamBreakChannel);

    useDashboardSensorOverridesEntry.setDefaultBoolean(false);
    intakeDetectedOverrideEntry.setDefaultBoolean(false);
    hopperDetectedOverrideEntry.setDefaultBoolean(false);
    shooterDetectedOverrideEntry.setDefaultBoolean(false);

    modeTimer.start();
  }

  @Override
  public void periodic() {
    updateSensors();
    updateJamDetection();
    runMode();
    logState();
  }

  public void setShooterReadySupplier(BooleanSupplier shooterReadySupplier) {
    this.shooterReadySupplier = shooterReadySupplier != null ? shooterReadySupplier : () -> false;
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

  public Command feedWhileHeldCommand() {
    return Commands.startEnd(() -> requestMode(Mode.FEED), () -> requestMode(Mode.HOLD), this);
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
        if (!hasAnyBeamBreakConfigured()
            && modeTimer.hasElapsed(GamePieceManagerConstants.sensorlessCollectToHoldSeconds)) {
          requestMode(Mode.HOLD);
          applyHold();
          break;
        }
        applyCollect();
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
    intakeDetected = readBeamBreak(intakeBeamBreak, intakeDetectedOverrideEntry, useOverrides);
    hopperDetected = readBeamBreak(hopperBeamBreak, hopperDetectedOverrideEntry, useOverrides);
    shooterDetected = readBeamBreak(shooterBeamBreak, shooterDetectedOverrideEntry, useOverrides);
  }

  private void updateJamDetection() {
    if (mode == Mode.COLLECT || mode == Mode.FEED) {
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
    hopper.setTargetBeltSpeed(GamePieceManagerConstants.collectHopperSpeed);
    agitators.setTargetAgitatorSpeed(GamePieceManagerConstants.collectAgitatorSpeed);
    intake.updateIntake();
    hopper.updateBelt();
    agitators.updateAgitators();
  }

  private void applyHold() {
    stopFeedMotors();
  }

  private void applyFeed() {
    intake.stopIntake();
    hopper.setTargetBeltSpeed(GamePieceManagerConstants.feedHopperSpeed);
    agitators.setTargetAgitatorSpeed(GamePieceManagerConstants.feedAgitatorSpeed);
    hopper.updateBelt();
    agitators.updateAgitators();
  }

  private void applyReverse() {
    intake.setTargetIntakeSpeed(GamePieceManagerConstants.reverseSpeed);
    hopper.setTargetBeltSpeed(GamePieceManagerConstants.reverseSpeed);
    agitators.setTargetAgitatorSpeed(GamePieceManagerConstants.reverseSpeed);
    intake.updateIntake();
    hopper.updateBelt();
    agitators.updateAgitators();
  }

  private void stopFeedMotors() {
    intake.stopIntake();
    hopper.stopBelt();
    agitators.stopAgitators();
  }

  private boolean hasMeasuredStagedPiece() {
    return shooterDetected || hopperDetected;
  }

  private boolean hasPieceAvailableForFeed() {
    return hasAnyBeamBreakConfigured() ? hasMeasuredStagedPiece() : true;
  }

  private boolean hasAnyBeamBreakConfigured() {
    return intakeBeamBreak != null || hopperBeamBreak != null || shooterBeamBreak != null;
  }

  private double getFeedCurrentAmps() {
    return intake.getIntakeDriveCurrentAmps()
        + hopper.getHopperBeltCurrentAmps()
        + agitators.getAverageAgitatorCurrentAmps();
  }

  private static DigitalInput createBeamBreak(int channel) {
    return channel >= 0 ? new DigitalInput(channel) : null;
  }

  private static boolean decodeBeamBreak(DigitalInput sensor) {
    boolean sensorActive = sensor.get();
    return GamePieceManagerConstants.beamBreakActiveLow ? !sensorActive : sensorActive;
  }

  private static boolean readBeamBreak(
      DigitalInput sensor, NetworkTableEntry overrideEntry, boolean useOverrides) {
    if (sensor != null) {
      return decodeBeamBreak(sensor);
    }
    return useOverrides && overrideEntry.getBoolean(false);
  }

  private void logState() {
    Logger.recordOutput("GamePieceManager/Mode", mode.name());
    Logger.recordOutput("GamePieceManager/Sensor/IntakeDetected", intakeDetected);
    Logger.recordOutput("GamePieceManager/Sensor/HopperDetected", hopperDetected);
    Logger.recordOutput("GamePieceManager/Sensor/ShooterDetected", shooterDetected);
    Logger.recordOutput(
        "GamePieceManager/Interlock/ShooterReady", shooterReadySupplier.getAsBoolean());
    Logger.recordOutput(
        "GamePieceManager/Interlock/PieceAvailableForFeed", hasPieceAvailableForFeed());
    Logger.recordOutput("GamePieceManager/Current/FeedCurrentAmps", getFeedCurrentAmps());
  }
}
