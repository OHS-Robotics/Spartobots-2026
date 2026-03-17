package frc.robot.operator;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.AutoRoutines;
import frc.robot.util.NetworkTablesUtil;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class OperatorDashboard {
  private final OperatorActionPublisher actionPublisher;
  private final LoggedDashboardChooser<Command> autoChooser;
  private final AutoRoutines autoRoutines;
  private final NetworkTable chooserStateTable =
      NetworkTablesUtil.state("Operator").getSubTable("AutoChooser");
  private final NetworkTableEntry selectedAutoEntry = chooserStateTable.getEntry("Selected");
  private final NetworkTableEntry autoOptionsEntry = chooserStateTable.getEntry("Options");

  public OperatorDashboard(
      OperatorActionPublisher actionPublisher,
      LoggedDashboardChooser<Command> autoChooser,
      AutoRoutines autoRoutines) {
    this.actionPublisher = actionPublisher;
    this.autoChooser = autoChooser;
    this.autoRoutines = autoRoutines;
    autoOptionsEntry.setStringArray(autoRoutines.getAutoOptionNames());
  }

  public void registerTrackedAction(String dashboardName, Command trackedCommand) {
    SmartDashboard.putData(dashboardName, trackedCommand);
  }

  public void registerAction(String dashboardName, String actionPath, Command command) {
    SmartDashboard.putData(dashboardName, actionPublisher.trackAction(actionPath, command));
  }

  public void periodic() {
    selectedAutoEntry.setString(autoRoutines.getSelectedAutoName(autoChooser.get()));
    autoOptionsEntry.setStringArray(autoRoutines.getAutoOptionNames());
  }
}
