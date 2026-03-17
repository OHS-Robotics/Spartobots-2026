package frc.robot.operator;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.NetworkTablesUtil;
import java.util.HashMap;
import java.util.Map;

public class OperatorActionPublisher {
  private final NetworkTable actionsTable = NetworkTablesUtil.actions("Operator");
  private final Map<String, ActionState> actionStates = new HashMap<>();

  public void registerAction(String actionPath) {
    actionState(actionPath).availableEntry.setBoolean(true);
  }

  public void markTriggered(String actionPath) {
    ActionState state = actionState(actionPath);
    state.availableEntry.setBoolean(true);
    state.scheduledEntry.setBoolean(true);
    state.runningEntry.setBoolean(false);
    state.lastTriggeredTimestampSecondsEntry.setDouble(Timer.getFPGATimestamp());
  }

  public Command trackAction(String actionPath, Command command) {
    registerAction(actionPath);
    return command
        .beforeStarting(
            () -> {
              ActionState state = actionState(actionPath);
              state.scheduledEntry.setBoolean(true);
              state.runningEntry.setBoolean(true);
              state.lastTriggeredTimestampSecondsEntry.setDouble(Timer.getFPGATimestamp());
            })
        .finallyDo(
            () -> {
              ActionState state = actionState(actionPath);
              state.scheduledEntry.setBoolean(false);
              state.runningEntry.setBoolean(false);
            });
  }

  private ActionState actionState(String actionPath) {
    return actionStates.computeIfAbsent(actionPath, this::createActionState);
  }

  private ActionState createActionState(String actionPath) {
    NetworkTable actionTable = descend(actionsTable, actionPath).getSubTable("State");
    ActionState state =
        new ActionState(
            actionTable.getEntry("Available"),
            actionTable.getEntry("Scheduled"),
            actionTable.getEntry("Running"),
            actionTable.getEntry("LastTriggeredTimestampSeconds"));
    state.availableEntry.setBoolean(true);
    state.scheduledEntry.setBoolean(false);
    state.runningEntry.setBoolean(false);
    state.lastTriggeredTimestampSecondsEntry.setDouble(Double.NaN);
    return state;
  }

  private static NetworkTable descend(NetworkTable table, String path) {
    NetworkTable current = table;
    for (String segment : path.split("/")) {
      if (!segment.isEmpty()) {
        current = current.getSubTable(segment);
      }
    }
    return current;
  }

  private record ActionState(
      NetworkTableEntry availableEntry,
      NetworkTableEntry scheduledEntry,
      NetworkTableEntry runningEntry,
      NetworkTableEntry lastTriggeredTimestampSecondsEntry) {}
}
