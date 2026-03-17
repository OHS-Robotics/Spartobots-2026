package frc.robot.operator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.NetworkTablesUtil;
import org.littletonrobotics.junction.Logger;

public class OperatorFeedbackController {
  private static final double endgameWindowStartSeconds = 30.0;
  private static final double endgameRumbleDurationSeconds = 0.65;
  private static final double endgameRumbleStrength = 0.65;

  private final GenericHID driverHid;

  private double endgameRumbleUntilTimestampSeconds = 0.0;
  private double impactRumbleUntilTimestampSeconds = 0.0;
  private double impactRumbleStrength = 0.0;
  private boolean endgameWindowLatched = false;

  public OperatorFeedbackController(GenericHID driverHid) {
    this.driverHid = driverHid;
  }

  public void periodic() {
    updateEndgameWindowState();
    updateDriverRumbleOutput(Timer.getFPGATimestamp());
  }

  public void triggerImpactRumble(double durationSeconds, double strength) {
    double untilTimestampSeconds = Timer.getFPGATimestamp() + durationSeconds;
    impactRumbleUntilTimestampSeconds =
        Math.max(impactRumbleUntilTimestampSeconds, untilTimestampSeconds);
    impactRumbleStrength = Math.max(impactRumbleStrength, strength);
  }

  public void onDisabledInit() {
    clearTransientState();
  }

  public void clearTransientState() {
    endgameWindowLatched = false;
    endgameRumbleUntilTimestampSeconds = 0.0;
    impactRumbleUntilTimestampSeconds = 0.0;
    impactRumbleStrength = 0.0;
    driverHid.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
  }

  private void updateEndgameWindowState() {
    double matchTimeSeconds = DriverStation.getMatchTime();
    boolean endgameWindowActive =
        DriverStation.isTeleopEnabled()
            && matchTimeSeconds >= 0.0
            && matchTimeSeconds <= endgameWindowStartSeconds;

    if (endgameWindowActive && !endgameWindowLatched) {
      endgameRumbleUntilTimestampSeconds = Timer.getFPGATimestamp() + endgameRumbleDurationSeconds;
    }

    if (!DriverStation.isTeleopEnabled()) {
      endgameWindowLatched = false;
      endgameRumbleUntilTimestampSeconds = 0.0;
    } else {
      endgameWindowLatched = endgameWindowActive;
    }

    Logger.recordOutput(
        NetworkTablesUtil.logPath("Match/State/EndgameWindowActive"), endgameWindowActive);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Match/State/TimeRemainingSeconds"), matchTimeSeconds);
  }

  private void updateDriverRumbleOutput(double nowSeconds) {
    if (nowSeconds >= impactRumbleUntilTimestampSeconds) {
      impactRumbleStrength = 0.0;
    }

    double rumbleStrength = 0.0;
    if (nowSeconds < endgameRumbleUntilTimestampSeconds) {
      rumbleStrength = Math.max(rumbleStrength, endgameRumbleStrength);
    }
    if (nowSeconds < impactRumbleUntilTimestampSeconds) {
      rumbleStrength = Math.max(rumbleStrength, impactRumbleStrength);
    }

    driverHid.setRumble(GenericHID.RumbleType.kBothRumble, rumbleStrength);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Operator/DriverFeedback/State/RumbleStrength"), rumbleStrength);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Operator/DriverFeedback/State/EndgameRumbleActive"),
        nowSeconds < endgameRumbleUntilTimestampSeconds);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Operator/DriverFeedback/State/ImpactRumbleActive"),
        nowSeconds < impactRumbleUntilTimestampSeconds);
  }
}
