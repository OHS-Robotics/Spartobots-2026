package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class DriveOdometryTest {
  @BeforeEach
  void setUp() {
    AutoBuilder.resetForTesting();
  }

  @Test
  void periodicUsesGyroHeadingWithoutAdditionalSignFlip() {
    Rotation2d measuredHeading = Rotation2d.fromDegrees(90.0);
    Drive drive =
        new Drive(
            new TestGyroIO(measuredHeading),
            new TestModuleIO(),
            new TestModuleIO(),
            new TestModuleIO(),
            new TestModuleIO());

    drive.periodic();

    assertEquals(measuredHeading.getDegrees(), drive.getPose().getRotation().getDegrees(), 1e-9);
  }

  private static class TestGyroIO implements GyroIO {
    private final Rotation2d measuredHeading;

    private TestGyroIO(Rotation2d measuredHeading) {
      this.measuredHeading = measuredHeading;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
      inputs.connected = true;
      inputs.yawPosition = measuredHeading;
      inputs.odometryYawTimestamps = new double[] {0.02};
      inputs.odometryYawPositions = new Rotation2d[] {measuredHeading};
    }
  }

  private static class TestModuleIO implements ModuleIO {
    @Override
    public void updateInputs(ModuleIOInputs inputs) {
      inputs.driveConnected = true;
      inputs.turnConnected = true;
      inputs.turnPosition = Rotation2d.kZero;
      inputs.odometryTimestamps = new double[] {0.02};
      inputs.odometryDrivePositionsRad = new double[] {0.0};
      inputs.odometryTurnPositions = new Rotation2d[] {Rotation2d.kZero};
    }
  }
}
