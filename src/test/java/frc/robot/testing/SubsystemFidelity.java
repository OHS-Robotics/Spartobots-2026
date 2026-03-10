package frc.robot.testing;

/**
 * Classifies how much confidence simulator results provide for a subsystem.
 *
 * <p>Use this to keep placeholder mechanism behavior separate from hardware-backed parity claims.
 */
public enum SubsystemFidelity {
  HARDWARE_PARITY,
  SIM_ONLY,
  PLACEHOLDER;

  public boolean requiresHardwareValidation() {
    return this != HARDWARE_PARITY;
  }
}
