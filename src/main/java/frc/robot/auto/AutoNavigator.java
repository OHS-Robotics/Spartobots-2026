package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public interface AutoNavigator {
  Command navigateTo(Pose2d targetPose, AutoSpec.RiskTier riskTier);
}
