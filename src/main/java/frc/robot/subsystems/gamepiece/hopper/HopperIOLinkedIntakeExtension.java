package frc.robot.subsystems.gamepiece.hopper;

import frc.robot.subsystems.gamepiece.intake.Intake;

/**
 * Hopper extension shim for the current robot, where the hopper extension is mechanically tied to
 * the intake extension and does not have its own dedicated actuator.
 */
public class HopperIOLinkedIntakeExtension implements HopperIO {
  private final Intake intake;
  private double lastAppliedOutput = 0.0;

  public HopperIOLinkedIntakeExtension(Intake intake) {
    this.intake = intake;
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    inputs.extensionConnected = true;
    inputs.extensionPositionRotations = intake.getIntakePivotMeasuredPositionRotations();
    inputs.extensionVelocityRpm = 0.0;
    inputs.extensionAppliedOutput = lastAppliedOutput;
    inputs.extensionCurrentAmps = intake.getIntakePivotCurrentAmps();
  }

  @Override
  public void setExtensionOutput(double output) {
    lastAppliedOutput = output;
    intake.setIntakePivotSpeed(output);
  }

  @Override
  public void resetSimulationState() {
    lastAppliedOutput = 0.0;
  }
}
