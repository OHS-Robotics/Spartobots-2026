package frc.robot.operator;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.NetworkTablesUtil;
import java.util.Locale;
import org.littletonrobotics.junction.Logger;

public class DriverStationHidCharacterization {
  private final GenericHID driverHid;
  private final GenericHID operatorHid;

  public DriverStationHidCharacterization(GenericHID driverHid, GenericHID operatorHid) {
    this.driverHid = driverHid;
    this.operatorHid = operatorHid;
  }

  public void periodic() {
    publishHidState("Driver", driverHid);
    publishHidState("Operator", operatorHid);
  }

  private void publishHidState(String label, GenericHID hid) {
    String dashboardPrefix = "Characterization/" + label;
    String logPrefix = NetworkTablesUtil.logPath("Operator/DriverStationCharacterization/" + label);

    double[] axisValues = readAxisValues(hid);
    double[] povValues = readPovValues(hid);
    double[] pressedButtonNumbers = readPressedButtonNumbers(hid);

    SmartDashboard.putBoolean(dashboardPrefix + "/Connected", hid.isConnected());
    SmartDashboard.putString(dashboardPrefix + "/Name", hid.getName());
    SmartDashboard.putString(dashboardPrefix + "/Type", String.valueOf(hid.getType()));
    SmartDashboard.putNumber(dashboardPrefix + "/Port", hid.getPort());
    SmartDashboard.putNumber(dashboardPrefix + "/AxisCount", axisValues.length);
    SmartDashboard.putNumber(dashboardPrefix + "/POVCount", povValues.length);
    SmartDashboard.putNumber(dashboardPrefix + "/ButtonCount", hid.getButtonCount());
    SmartDashboard.putString(dashboardPrefix + "/Axes", formatAxisSummary(axisValues));
    SmartDashboard.putString(dashboardPrefix + "/POVs", formatPovSummary(povValues));
    SmartDashboard.putString(
        dashboardPrefix + "/PressedButtons", formatPressedButtonSummary(pressedButtonNumbers));

    Logger.recordOutput(logPrefix + "/Connected", hid.isConnected());
    Logger.recordOutput(logPrefix + "/Name", hid.getName());
    Logger.recordOutput(logPrefix + "/Type", String.valueOf(hid.getType()));
    Logger.recordOutput(logPrefix + "/Port", hid.getPort());
    Logger.recordOutput(logPrefix + "/AxisCount", axisValues.length);
    Logger.recordOutput(logPrefix + "/POVCount", povValues.length);
    Logger.recordOutput(logPrefix + "/ButtonCount", hid.getButtonCount());
    Logger.recordOutput(logPrefix + "/Axes/Values", axisValues);
    Logger.recordOutput(logPrefix + "/POVs/Values", povValues);
    Logger.recordOutput(logPrefix + "/Buttons/PressedNumbers", pressedButtonNumbers);
    Logger.recordOutput(logPrefix + "/Axes/Summary", formatAxisSummary(axisValues));
    Logger.recordOutput(logPrefix + "/POVs/Summary", formatPovSummary(povValues));
    Logger.recordOutput(
        logPrefix + "/Buttons/Summary", formatPressedButtonSummary(pressedButtonNumbers));
  }

  private static double[] readAxisValues(GenericHID hid) {
    double[] axisValues = new double[hid.getAxisCount()];
    for (int axis = 0; axis < axisValues.length; axis++) {
      axisValues[axis] = hid.getRawAxis(axis);
    }
    return axisValues;
  }

  private static double[] readPovValues(GenericHID hid) {
    double[] povValues = new double[hid.getPOVCount()];
    for (int pov = 0; pov < povValues.length; pov++) {
      povValues[pov] = hid.getPOV(pov);
    }
    return povValues;
  }

  private static double[] readPressedButtonNumbers(GenericHID hid) {
    int pressedButtonCount = 0;
    for (int button = 1; button <= hid.getButtonCount(); button++) {
      if (hid.getRawButton(button)) {
        pressedButtonCount++;
      }
    }

    double[] pressedButtonNumbers = new double[pressedButtonCount];
    int pressedButtonIndex = 0;
    for (int button = 1; button <= hid.getButtonCount(); button++) {
      if (hid.getRawButton(button)) {
        pressedButtonNumbers[pressedButtonIndex++] = button;
      }
    }
    return pressedButtonNumbers;
  }

  private static String formatAxisSummary(double[] axisValues) {
    if (axisValues.length == 0) {
      return "none";
    }

    StringBuilder summary = new StringBuilder();
    for (int axis = 0; axis < axisValues.length; axis++) {
      if (summary.length() > 0) {
        summary.append(", ");
      }
      summary.append(axis).append('=').append(String.format(Locale.US, "%.3f", axisValues[axis]));
    }
    return summary.toString();
  }

  private static String formatPovSummary(double[] povValues) {
    if (povValues.length == 0) {
      return "none";
    }

    StringBuilder summary = new StringBuilder();
    for (int pov = 0; pov < povValues.length; pov++) {
      if (summary.length() > 0) {
        summary.append(", ");
      }
      summary.append(pov).append('=').append((int) povValues[pov]);
    }
    return summary.toString();
  }

  private static String formatPressedButtonSummary(double[] pressedButtonNumbers) {
    if (pressedButtonNumbers.length == 0) {
      return "none";
    }

    StringBuilder summary = new StringBuilder();
    for (double pressedButtonNumber : pressedButtonNumbers) {
      if (summary.length() > 0) {
        summary.append(", ");
      }
      summary.append((int) pressedButtonNumber);
    }
    return summary.toString();
  }
}
