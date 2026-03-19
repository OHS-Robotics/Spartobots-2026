package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.NetworkTablesUtil;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class TunableHolonomicPathController implements PathFollowingController {
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController rotationController;
  private boolean enabled = true;
  private Supplier<Optional<Rotation2d>> rotationTargetOverride = Optional::empty;

  public TunableHolonomicPathController(
      PIDConstants translationConstants, PIDConstants rotationConstants, double periodSeconds) {
    xController =
        new PIDController(
            translationConstants.kP,
            translationConstants.kI,
            translationConstants.kD,
            periodSeconds);
    yController =
        new PIDController(
            translationConstants.kP,
            translationConstants.kI,
            translationConstants.kD,
            periodSeconds);
    rotationController =
        new PIDController(
            rotationConstants.kP, rotationConstants.kI, rotationConstants.kD, periodSeconds);

    setTranslationIntegratorRange(translationConstants.iZone);
    setRotationIntegratorRange(rotationConstants.iZone);
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void setEnabled(boolean enabled) {
    this.enabled = enabled;
  }

  public void setTranslationPID(double kp, double ki, double kd) {
    xController.setPID(kp, ki, kd);
    yController.setPID(kp, ki, kd);
  }

  public void setRotationPID(double kp, double ki, double kd) {
    rotationController.setPID(kp, ki, kd);
  }

  public void setRotationTargetOverride(Supplier<Optional<Rotation2d>> rotationTargetOverride) {
    this.rotationTargetOverride =
        rotationTargetOverride != null ? rotationTargetOverride : Optional::empty;
  }

  public void clearRotationTargetOverride() {
    rotationTargetOverride = Optional::empty;
  }

  public void setTranslationIntegratorRange(double iZone) {
    double clampedIZone = Math.abs(iZone);
    xController.setIntegratorRange(-clampedIZone, clampedIZone);
    yController.setIntegratorRange(-clampedIZone, clampedIZone);
  }

  public void setRotationIntegratorRange(double iZone) {
    double clampedIZone = Math.abs(iZone);
    rotationController.setIntegratorRange(-clampedIZone, clampedIZone);
  }

  @Override
  public ChassisSpeeds calculateRobotRelativeSpeeds(
      Pose2d currentPose, PathPlannerTrajectoryState targetState) {
    double xFeedforward = targetState.fieldSpeeds.vxMetersPerSecond;
    double yFeedforward = targetState.fieldSpeeds.vyMetersPerSecond;
    double rotationFeedforward = targetState.fieldSpeeds.omegaRadiansPerSecond;
    double xError = targetState.pose.getX() - currentPose.getX();
    double yError = targetState.pose.getY() - currentPose.getY();
    Optional<Rotation2d> rotationOverride = rotationTargetOverride.get();
    if (rotationOverride == null) {
      rotationOverride = Optional.empty();
    }
    Rotation2d targetRotation = rotationOverride.orElse(targetState.pose.getRotation());
    double rotationErrorRadians =
        MathUtil.angleModulus(targetRotation.getRadians() - currentPose.getRotation().getRadians());

    double xFeedback = 0.0;
    double yFeedback = 0.0;
    double rotationFeedback = 0.0;
    if (enabled) {
      xFeedback = xController.calculate(currentPose.getX(), targetState.pose.getX());
      yFeedback = yController.calculate(currentPose.getY(), targetState.pose.getY());
      rotationFeedback =
          rotationController.calculate(
              currentPose.getRotation().getRadians(), targetRotation.getRadians());
    }

    ChassisSpeeds feedforwardSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xFeedforward, yFeedforward, rotationFeedforward, currentPose.getRotation());
    ChassisSpeeds feedbackSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xFeedback, yFeedback, rotationFeedback, currentPose.getRotation());
    ChassisSpeeds outputSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xFeedforward + xFeedback,
            yFeedforward + yFeedback,
            rotationFeedforward + rotationFeedback,
            currentPose.getRotation());
    logPathFollowingState(
        targetState.pose,
        targetRotation,
        xError,
        yError,
        rotationErrorRadians,
        rotationOverride.isPresent(),
        feedforwardSpeeds,
        feedbackSpeeds,
        outputSpeeds);
    return outputSpeeds;
  }

  @Override
  public void reset(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
    xController.reset();
    yController.reset();
    rotationController.reset();
    logPathFollowingState(
        currentPose,
        currentPose.getRotation(),
        0.0,
        0.0,
        0.0,
        false,
        new ChassisSpeeds(),
        new ChassisSpeeds(),
        new ChassisSpeeds());
  }

  @Override
  public boolean isHolonomic() {
    return true;
  }

  private void logPathFollowingState(
      Pose2d targetPose,
      Rotation2d targetRotation,
      double xError,
      double yError,
      double rotationErrorRadians,
      boolean rotationOverrideActive,
      ChassisSpeeds feedforwardSpeeds,
      ChassisSpeeds feedbackSpeeds,
      ChassisSpeeds outputSpeeds) {
    Logger.recordOutput(NetworkTablesUtil.logPath("Drive/PathFollowing/Enabled"), enabled);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Drive/PathFollowing/RotationOverrideActive"),
        rotationOverrideActive);
    Logger.recordOutput(NetworkTablesUtil.logPath("Drive/PathFollowing/TargetPose"), targetPose);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Drive/PathFollowing/TargetRotationDegrees"),
        targetRotation.getDegrees());
    Logger.recordOutput(NetworkTablesUtil.logPath("Drive/PathFollowing/TranslationErrorX"), xError);
    Logger.recordOutput(NetworkTablesUtil.logPath("Drive/PathFollowing/TranslationErrorY"), yError);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Drive/PathFollowing/RotationErrorRadians"),
        rotationErrorRadians);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Drive/PathFollowing/RotationErrorDegrees"),
        Math.toDegrees(rotationErrorRadians));
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Drive/PathFollowing/FeedforwardSpeeds"), feedforwardSpeeds);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Drive/PathFollowing/FeedbackSpeeds"), feedbackSpeeds);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Drive/PathFollowing/OutputSpeeds"), outputSpeeds);
  }
}
