package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.util.NetworkTablesUtil;
import java.util.Optional;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class DriveRoutingTest {
  @BeforeEach
  void setUp() {
    AutoBuilder.resetForTesting();
    NetworkTablesUtil.tuningMode("Targeting/Hub")
        .getSubTable("AutoAim")
        .getEntry("MaxLinearAccelerationMetersPerSecSquared")
        .setDouble(DriveConstants.maxAccelerationMeterPerSecSquared);
  }

  @Test
  void selectsAllianceHubPoseByAlliance() {
    assertEquals(
        Constants.blueHub.getX(),
        Drive.selectAllianceHubPose(Optional.of(Alliance.Blue)).getX(),
        1e-9);
    assertEquals(
        Constants.redHub.getX(),
        Drive.selectAllianceHubPose(Optional.of(Alliance.Red)).getX(),
        1e-9);
  }

  @Test
  void selectsOutpostApproachPosesByAlliance() {
    var bluePoses = Drive.selectOutpostApproachPoses(Optional.of(Alliance.Blue));
    assertEquals(Constants.blueOutpostBefore.getX(), bluePoses[0].getX(), 1e-9);
    assertEquals(Constants.blueOutpost.getX(), bluePoses[1].getX(), 1e-9);

    var redPoses = Drive.selectOutpostApproachPoses(Optional.of(Alliance.Red));
    assertEquals(Constants.redOutpostBefore.getX(), redPoses[0].getX(), 1e-9);
    assertEquals(Constants.redOutpost.getX(), redPoses[1].getX(), 1e-9);
  }

  @Test
  void driveToOutpostCommandReturnsDeferredCommand() {
    Drive drive =
        new Drive(
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {});
    var command = drive.driveToOutpostCommand();
    assertNotNull(command);
    assertEquals("DeferredCommand", command.getClass().getSimpleName());
  }

  @Test
  void pathfindToTranslationReturnsDeferredCommand() {
    Drive drive =
        new Drive(
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {});
    var command = drive.pathfindToTranslation(new Translation2d(2.0, 3.0));
    assertNotNull(command);
    assertEquals("DeferredCommand", command.getClass().getSimpleName());
  }

  @Test
  void hubAutoAimAccelerationLimitLoadsFromNetworkTables() {
    Drive drive =
        new Drive(
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {});
    var entry =
        NetworkTablesUtil.tuningMode("Targeting/Hub")
            .getSubTable("AutoAim")
            .getEntry("MaxLinearAccelerationMetersPerSecSquared");

    entry.setDouble(2.25);
    drive.periodic();
    assertEquals(2.25, drive.getHubAutoAimLinearAccelerationLimitMetersPerSecSquared(), 1e-9);

    entry.setDouble(-1.0);
    drive.periodic();
    assertEquals(0.0, drive.getHubAutoAimLinearAccelerationLimitMetersPerSecSquared(), 1e-9);
    assertEquals(0.0, entry.getDouble(-999.0), 1e-9);
  }
}
