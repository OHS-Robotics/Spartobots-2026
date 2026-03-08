package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShotSolution;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public interface SuperstructureDrive {
  public Pose2d getPose();

  public ChassisSpeeds getChassisSpeeds();

  public Command faceTarget(
      DoubleSupplier xSupplier, DoubleSupplier ySupplier, Supplier<Rotation2d> targetHeading);

  public boolean isAimed(Supplier<Rotation2d> targetHeadingSupplier);

  public boolean isPoseTrusted();

  public Command holdShotPose(Supplier<ShotSolution> shotSolutionSupplier);
}
