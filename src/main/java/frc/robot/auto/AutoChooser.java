package frc.robot.auto;

import java.util.Objects;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public final class AutoChooser {
  private final LoggedDashboardChooser<AutoChoice> chooser;
  private final AutoChoice defaultChoice;

  public AutoChooser(String name, AutoChoice defaultChoice) {
    this.chooser = new LoggedDashboardChooser<>(name);
    this.defaultChoice = Objects.requireNonNull(defaultChoice, "defaultChoice");
    chooser.addDefaultOption(defaultChoice.label(), defaultChoice);
  }

  public void addChoice(AutoChoice choice) {
    if (!choice.label().equals(defaultChoice.label())) {
      chooser.addOption(choice.label(), choice);
    }
  }

  public AutoChoice getSelectedChoice() {
    AutoChoice selectedChoice = chooser.get();
    return selectedChoice != null ? selectedChoice : defaultChoice;
  }

  public AutoCommand getSelected(AutoRoutineFactory factory) {
    return getSelectedChoice().compose(factory);
  }
}
