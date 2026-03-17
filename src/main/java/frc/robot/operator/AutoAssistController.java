package frc.robot.operator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Supplier;

public class AutoAssistController {
  private final OperatorActionPublisher actionPublisher;
  private Command activeAutoAssistCommand = null;

  public AutoAssistController(OperatorActionPublisher actionPublisher) {
    this.actionPublisher = actionPublisher;
  }

  public Command scheduleAction(String actionPath, Supplier<Command> commandSupplier) {
    actionPublisher.registerAction(actionPath);
    return Commands.runOnce(() -> scheduleCommand(actionPath, commandSupplier.get()));
  }

  public void cancel() {
    if (activeAutoAssistCommand != null) {
      activeAutoAssistCommand.cancel();
      activeAutoAssistCommand = null;
    }
  }

  public void onDisabledInit() {
    cancel();
  }

  private void scheduleCommand(String actionPath, Command command) {
    cancel();
    if (command == null) {
      return;
    }

    actionPublisher.markTriggered(actionPath);
    final Command[] trackedCommand = new Command[1];
    trackedCommand[0] =
        actionPublisher
            .trackAction(
                actionPath,
                command.finallyDo(
                    () -> {
                      if (activeAutoAssistCommand == trackedCommand[0]) {
                        activeAutoAssistCommand = null;
                      }
                    }))
            .withName(command.getName());
    activeAutoAssistCommand = trackedCommand[0];
    CommandScheduler.getInstance().schedule(trackedCommand[0]);
  }
}
