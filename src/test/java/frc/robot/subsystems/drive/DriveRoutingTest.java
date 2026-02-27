package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import java.util.Optional;
import org.junit.jupiter.api.Test;

class DriveRoutingTest {
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
}
