package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public interface SuperstructureDrive {
  public Pose2d getPose();

  public Translation2d getFieldRelativeVelocity();

  public Command aimWhileDriving(
      DoubleSupplier xSupplier, DoubleSupplier ySupplier, Supplier<Rotation2d> targetHeading);
}
