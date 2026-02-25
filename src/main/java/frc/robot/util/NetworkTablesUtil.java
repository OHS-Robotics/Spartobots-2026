package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

/** Centralizes the NetworkTables hierarchy used across subsystems and commands. */
public final class NetworkTablesUtil {
  public static final String rootTableName = "Spartobots2026";

  private static final String subsystemsSection = "Subsystems";
  private static final String commandsSection = "Commands";
  private static final String tuningSection = "Tuning";
  private static final String telemetrySection = "Telemetry";
  private static final String commonSection = "Common";
  private static final String modesSection = "Modes";

  private NetworkTablesUtil() {}

  public static NetworkTable subsystemTable(String subsystemPath) {
    return descend(rootTable().getSubTable(subsystemsSection), subsystemPath);
  }

  public static NetworkTable commandTable(String commandPath) {
    return descend(rootTable().getSubTable(commandsSection), commandPath);
  }

  public static NetworkTable tuningCommonTable(NetworkTable table) {
    return table.getSubTable(tuningSection).getSubTable(commonSection);
  }

  public static NetworkTable tuningModeTable(NetworkTable table) {
    return tuningModeTable(table, Constants.currentMode);
  }

  public static NetworkTable tuningModeTable(NetworkTable table, Constants.Mode mode) {
    return table.getSubTable(tuningSection).getSubTable(modesSection).getSubTable(mode.name());
  }

  public static NetworkTable telemetryTable(NetworkTable subsystemTable) {
    return subsystemTable.getSubTable(telemetrySection);
  }

  public static NetworkTable sharedModeTuningTable(String path) {
    return descend(
        rootTable()
            .getSubTable(tuningSection)
            .getSubTable(modesSection)
            .getSubTable(Constants.currentMode.name()),
        path);
  }

  public static String absoluteKey(String path) {
    String trimmedPath = trimPath(path);
    return trimmedPath.isEmpty() ? "/" + rootTableName : "/" + rootTableName + "/" + trimmedPath;
  }

  private static NetworkTable rootTable() {
    return NetworkTableInstance.getDefault().getTable(rootTableName);
  }

  private static NetworkTable descend(NetworkTable base, String path) {
    NetworkTable table = base;
    String trimmedPath = trimPath(path);
    if (trimmedPath.isEmpty()) {
      return table;
    }

    String[] segments = trimmedPath.split("/");
    for (String segment : segments) {
      if (!segment.isEmpty()) {
        table = table.getSubTable(segment);
      }
    }
    return table;
  }

  private static String trimPath(String path) {
    if (path == null) {
      return "";
    }

    int start = 0;
    int end = path.length();
    while (start < end && path.charAt(start) == '/') {
      start++;
    }
    while (end > start && path.charAt(end - 1) == '/') {
      end--;
    }
    return path.substring(start, end);
  }
}
