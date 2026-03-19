package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class DriveCommandsTest {
  @BeforeEach
  void setUp() {
    AutoBuilder.resetForTesting();
  }

  @Test
  void linearAccelerationLimitClampsVelocityVectorStep() {
    Translation2d limitedVelocity =
        DriveCommands.applyLinearAccelerationLimit(
            Translation2d.kZero, new Translation2d(3.0, 4.0), 5.0, 0.02);

    assertEquals(0.06, limitedVelocity.getX(), 1e-9);
    assertEquals(0.08, limitedVelocity.getY(), 1e-9);
  }

  @Test
  void joystickDriveAtAngleAppliesConfiguredLinearAccelerationLimit() {
    RecordingDrive drive = new RecordingDrive();
    Command command =
        DriveCommands.joystickDriveAtAngle(
            drive, () -> 1.0, () -> 0.0, () -> Rotation2d.kZero, () -> 4.0);

    command.initialize();
    command.execute();
    command.execute();
    command.end(false);

    assertEquals(0.08, drive.getLastRequestedSpeeds().vxMetersPerSecond, 1e-9);
    assertEquals(0.0, drive.getLastRequestedSpeeds().vyMetersPerSecond, 1e-9);
    assertEquals(0.0, drive.getLastRequestedSpeeds().omegaRadiansPerSecond, 1e-9);
  }

  private static class RecordingDrive extends Drive {
    private ChassisSpeeds lastRequestedSpeeds = new ChassisSpeeds();

    RecordingDrive() {
      super(
          new GyroIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {});
    }

    @Override
    public void runVelocity(ChassisSpeeds speeds) {
      lastRequestedSpeeds = speeds;
    }

    @Override
    public Rotation2d getRotation() {
      return Rotation2d.kZero;
    }

    @Override
    public Translation2d getFieldRelativeVelocityMetersPerSecond() {
      return Translation2d.kZero;
    }

    ChassisSpeeds getLastRequestedSpeeds() {
      return lastRequestedSpeeds;
    }
  }
}
