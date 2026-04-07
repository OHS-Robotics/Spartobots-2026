package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
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
    TestGyroIO gyro = new TestGyroIO(measuredHeading);
    Drive drive =
        new Drive(
            gyro, new TestModuleIO(), new TestModuleIO(), new TestModuleIO(), new TestModuleIO());

    gyro.setReportedHeading(measuredHeading);
    drive.periodic();

    assertEquals(measuredHeading.getDegrees(), drive.getPose().getRotation().getDegrees(), 1e-9);
  }

  @Test
  void setPoseReseedsGyroHeadingForFutureOdometryLoops() {
    TestGyroIO gyro = new TestGyroIO(Rotation2d.fromDegrees(90.0));
    Drive drive =
        new Drive(
            gyro, new TestModuleIO(), new TestModuleIO(), new TestModuleIO(), new TestModuleIO());

    drive.periodic();

    Pose2d relocalizedPose = new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(15.0));
    drive.setPose(relocalizedPose);
    drive.periodic();

    assertEquals(
        relocalizedPose.getRotation().getDegrees(), gyro.getLastSetAngle().getDegrees(), 1e-9);
    assertEquals(relocalizedPose.getX(), drive.getPose().getX(), 1e-9);
    assertEquals(relocalizedPose.getY(), drive.getPose().getY(), 1e-9);
    assertEquals(
        relocalizedPose.getRotation().getDegrees(),
        drive.getPose().getRotation().getDegrees(),
        1e-9);
  }

  private static class TestGyroIO implements GyroIO {
    private Rotation2d measuredHeading;
    private Rotation2d lastSetAngle = Rotation2d.kZero;
    private double timestampSeconds = 0.0;

    private TestGyroIO(Rotation2d measuredHeading) {
      this.measuredHeading = measuredHeading;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
      inputs.connected = true;
      inputs.yawPosition = measuredHeading;
      timestampSeconds += 0.02;
      inputs.odometryYawTimestamps = new double[] {timestampSeconds};
      inputs.odometryYawPositions = new Rotation2d[] {measuredHeading};
    }

    @Override
    public void setAngle(Rotation2d angle) {
      lastSetAngle = angle;
      measuredHeading = angle;
    }

    private Rotation2d getLastSetAngle() {
      return lastSetAngle;
    }

    private void setReportedHeading(Rotation2d heading) {
      measuredHeading = heading;
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
