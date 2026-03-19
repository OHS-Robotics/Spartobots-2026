package frc.robot.subsystems.gamepiece;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.subsystems.gamepiece.hopper.Hopper;
import frc.robot.subsystems.gamepiece.hopper.HopperConstants;
import frc.robot.subsystems.gamepiece.hopper.HopperIO;
import frc.robot.subsystems.gamepiece.indexers.Indexers;
import frc.robot.subsystems.gamepiece.indexers.IndexersConstants;
import frc.robot.subsystems.gamepiece.indexers.IndexersIO;
import frc.robot.subsystems.gamepiece.intake.Intake;
import frc.robot.subsystems.gamepiece.intake.IntakeConstants;
import frc.robot.subsystems.gamepiece.intake.IntakeIO;
import frc.robot.util.NetworkTablesUtil;
import org.junit.jupiter.api.Test;

class MechanismCalibrationModeTest {
  @Test
  void intakeCalibrationModeUsesNetworkTableClosedLoopSetpoints() {
    FakeIntakeIO io = new FakeIntakeIO();
    Intake intake = new Intake(io);
    var calibrationTable =
        NetworkTablesUtil.tuningCommon(NetworkTablesUtil.domain(IntakeConstants.configTableName))
            .getSubTable("Calibration");

    calibrationTable.getEntry("Drive/VelocitySetpointRotationsPerSec").setDouble(18.0);
    calibrationTable.getEntry("Pivot/PositionSetpointRotations").setDouble(0.75);
    calibrationTable.getEntry("Enabled").setBoolean(true);
    intake.periodic();

    assertTrue(intake.isCalibrationModeEnabled());
    assertEquals(18.0, io.driveVelocitySetpointRotationsPerSec, 1e-9);
    assertEquals(0.75, io.pivotPositionSetpointRotations, 1e-9);
  }

  @Test
  void hopperCalibrationModeUsesNetworkTableClosedLoopSetpoints() {
    FakeHopperIO io = new FakeHopperIO();
    Hopper hopper = new Hopper(io);
    var calibrationTable =
        NetworkTablesUtil.tuningCommon(NetworkTablesUtil.domain(HopperConstants.configTableName))
            .getSubTable("Calibration");

    calibrationTable.getEntry("Agitator/VelocitySetpointRotationsPerSec").setDouble(7.5);
    calibrationTable.getEntry("Extension/PositionSetpointRotations").setDouble(0.85);
    calibrationTable.getEntry("Enabled").setBoolean(true);
    hopper.periodic();

    assertTrue(hopper.isCalibrationModeEnabled());
    assertEquals(7.5, io.agitatorVelocitySetpointRotationsPerSec, 1e-9);
    assertEquals(0.85, io.extensionPositionSetpointRotations, 1e-9);
  }

  @Test
  void indexerCalibrationModeUsesNetworkTableClosedLoopSetpoints() {
    FakeIndexersIO io = new FakeIndexersIO();
    Indexers indexers = new Indexers(io);
    var calibrationTable =
        NetworkTablesUtil.tuningCommon(NetworkTablesUtil.domain(IndexersConstants.configTableName))
            .getSubTable("Calibration");

    calibrationTable.getEntry("Top/VelocitySetpointRotationsPerSec").setDouble(5.0);
    calibrationTable.getEntry("Bottom/VelocitySetpointRotationsPerSec").setDouble(-6.0);
    calibrationTable.getEntry("Enabled").setBoolean(true);
    indexers.periodic();

    assertTrue(indexers.isCalibrationModeEnabled());
    assertEquals(5.0, io.topVelocitySetpointRotationsPerSec, 1e-9);
    assertEquals(-6.0, io.bottomVelocitySetpointRotationsPerSec, 1e-9);
  }

  private static class FakeIntakeIO implements IntakeIO {
    double driveVelocitySetpointRotationsPerSec = 0.0;
    double pivotPositionSetpointRotations =
        IntakeConstants.defaultIntakePivotRetractedPositionRotations;

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
      inputs.driveConnected = true;
      inputs.pivotConnected = true;
      inputs.pivotPositionRotations = pivotPositionSetpointRotations;
    }

    @Override
    public void setDriveVelocitySetpointRotationsPerSec(double velocityRotationsPerSec) {
      driveVelocitySetpointRotationsPerSec = velocityRotationsPerSec;
    }

    @Override
    public void setPivotPositionSetpointRotations(double positionRotations) {
      pivotPositionSetpointRotations = positionRotations;
    }
  }

  private static class FakeHopperIO implements HopperIO {
    double agitatorVelocitySetpointRotationsPerSec = 0.0;
    double extensionPositionSetpointRotations =
        HopperConstants.defaultHopperExtensionRetractedPositionRotations;

    @Override
    public void updateInputs(HopperIOInputs inputs) {
      inputs.agitatorConnected = true;
      inputs.extensionConnected = true;
      inputs.extensionPositionRotations = extensionPositionSetpointRotations;
    }

    @Override
    public void setAgitatorVelocitySetpointRotationsPerSec(double velocityRotationsPerSec) {
      agitatorVelocitySetpointRotationsPerSec = velocityRotationsPerSec;
    }

    @Override
    public void setExtensionPositionSetpointRotations(double positionRotations) {
      extensionPositionSetpointRotations = positionRotations;
    }
  }

  private static class FakeIndexersIO implements IndexersIO {
    double topVelocitySetpointRotationsPerSec = 0.0;
    double bottomVelocitySetpointRotationsPerSec = 0.0;

    @Override
    public void updateInputs(IndexersIOInputs inputs) {
      inputs.topConnected = true;
      inputs.bottomConnected = true;
    }

    @Override
    public void setTopVelocitySetpointRotationsPerSec(double velocityRotationsPerSec) {
      topVelocitySetpointRotationsPerSec = velocityRotationsPerSec;
    }

    @Override
    public void setBottomVelocitySetpointRotationsPerSec(double velocityRotationsPerSec) {
      bottomVelocitySetpointRotationsPerSec = velocityRotationsPerSec;
    }
  }
}
