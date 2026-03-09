package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;

public class AdStarAutoNavigator implements AutoNavigator {
  private final Drive drive;

  public AdStarAutoNavigator(Drive drive) {
    this.drive = drive;
  }

  @Override
  public Command navigateTo(Pose2d targetPose, AutoRisk risk) {
    return AutoBuilder.pathfindToPose(targetPose, constraintsFor(risk))
        .withName("AUTO_NAVIGATE_" + risk.name());
  }

  private PathConstraints constraintsFor(AutoRisk risk) {
    double scale = AutoRuntimePolicy.speedScale(risk);
    return new PathConstraints(
        drive.getMaxLinearSpeedMetersPerSec() * scale,
        DriveConstants.maxAccelerationMeterPerSecSquared * scale,
        DriveConstants.maxRotationalSpeedRadiansPerSec * scale,
        DriveConstants.maxRotationalAccelerationRadiansPerSecSquared * scale);
  }
}
