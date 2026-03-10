package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.auto.AutoChoice;
import frc.robot.auto.AutoRoutineFactory;
import org.junit.jupiter.api.Test;

class RobotContainerCapabilityTest {
  @Test
  void unsupportedCapabilityMessageListsPlaceholderMechanismsAndDisabledFeatures() {
    RobotCapabilities capabilities = new RobotCapabilities(true, true, false, false, false, false);

    assertEquals(
        "Real robot is using placeholder mechanisms for intake, indexer, shooter, endgame. "
            + "Disabled features: acquire, hub shot, outpost feed, quick park, match autos.",
        RobotContainer.buildUnsupportedCapabilitiesMessage(capabilities));
  }

  @Test
  void unsupportedCapabilityMessageFallsBackToHardwareBackedMessage() {
    RobotCapabilities capabilities = new RobotCapabilities(true, true, true, true, true, true);

    assertEquals(
        "All configured robot mechanisms are hardware-backed.",
        RobotContainer.buildUnsupportedCapabilitiesMessage(capabilities));
  }

  @Test
  void defaultAutoFallsBackToDoNothingWhenMatchAutosAreUnsupported() {
    RobotCapabilities capabilities = new RobotCapabilities(true, true, false, false, false, false);

    AutoChoice choice =
        RobotContainer.defaultAutoChoiceFor(capabilities, new AutoRoutineFactory(null, null, null));

    assertEquals("Do Nothing | Match mechanisms unavailable", choice.label());
    assertTrue(choice.spec().isEmpty());
  }

  @Test
  void defaultAutoUsesFactoryLibraryWhenMatchAutosAreSupported() {
    RobotCapabilities capabilities = new RobotCapabilities(true, true, true, true, true, true);
    AutoRoutineFactory factory = new AutoRoutineFactory(null, null, null);

    AutoChoice choice = RobotContainer.defaultAutoChoiceFor(capabilities, factory);

    assertEquals(factory.defaultAutoChoice().label(), choice.label());
    assertTrue(choice.spec().isPresent());
  }
}
