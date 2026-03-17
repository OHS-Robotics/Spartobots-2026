package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

/** Centralizes the NetworkTables hierarchy used across subsystems and commands. */
public final class NetworkTablesUtil {
  public static final String rootTableName = "Spartobots2026";

  private static final String tuningSection = "Tuning";
  private static final String telemetrySection = "Telemetry";
  private static final String stateSection = "State";
  private static final String actionsSection = "Actions";
  private static final String commonSection = "Common";
  private static final String modesSection = "Modes";

  private NetworkTablesUtil() {}

  public static NetworkTable domain(String domainPath) {
    return descend(rootTable(), domainPath);
  }

  public static NetworkTable tuningCommon(String domainPath) {
    return tuningCommon(domain(domainPath));
  }

  public static NetworkTable tuningCommon(NetworkTable table) {
    return table.getSubTable(tuningSection).getSubTable(commonSection);
  }

  public static NetworkTable tuningMode(String domainPath) {
    return tuningMode(domain(domainPath));
  }

  public static NetworkTable tuningMode(String domainPath, Constants.Mode mode) {
    return tuningMode(domain(domainPath), mode);
  }

  public static NetworkTable tuningMode(NetworkTable table) {
    return tuningMode(table, Constants.currentMode);
  }

  public static NetworkTable tuningMode(NetworkTable table, Constants.Mode mode) {
    return table.getSubTable(tuningSection).getSubTable(modesSection).getSubTable(mode.name());
  }

  public static NetworkTable telemetry(String domainPath) {
    return telemetry(domain(domainPath));
  }

  public static NetworkTable telemetry(NetworkTable table) {
    return table.getSubTable(telemetrySection);
  }

  public static NetworkTable state(String domainPath) {
    return state(domain(domainPath));
  }

  public static NetworkTable state(NetworkTable table) {
    return table.getSubTable(stateSection);
  }

  public static NetworkTable actions(String domainPath) {
    return actions(domain(domainPath));
  }

  public static NetworkTable actions(NetworkTable table) {
    return table.getSubTable(actionsSection);
  }

  public static String absoluteKey(String path) {
    String trimmedPath = trimPath(path);
    return trimmedPath.isEmpty() ? "/" + rootTableName : "/" + rootTableName + "/" + trimmedPath;
  }

  public static String logPath(String path) {
    String trimmedPath = trimPath(path);
    return trimmedPath.isEmpty() ? rootTableName : rootTableName + "/" + trimmedPath;
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
