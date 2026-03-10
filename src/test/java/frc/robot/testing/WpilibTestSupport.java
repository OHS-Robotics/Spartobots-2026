package frc.robot.testing;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** Shared WPILib test lifecycle helpers for scheduler- and timer-driven scenarios. */
public final class WpilibTestSupport {
  private static boolean halInitialized = false;

  private WpilibTestSupport() {}

  public static void initializeHal() {
    if (!halInitialized) {
      HAL.initialize(500, 0);
      halInitialized = true;
    }
  }

  public static void resetSchedulerAndTime() {
    initializeHal();
    CommandScheduler.getInstance().cancelAll();
    DriverStationSim.resetData();
    SimHooks.pauseTiming();
    SimHooks.restartTiming();
  }

  public static void stepScheduler(double seconds) {
    SimHooks.stepTiming(seconds);
    CommandScheduler.getInstance().run();
  }
}
