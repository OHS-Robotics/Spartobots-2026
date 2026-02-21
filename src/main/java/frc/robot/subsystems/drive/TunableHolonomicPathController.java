package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class TunableHolonomicPathController implements PathFollowingController {
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController rotationController;
  private boolean enabled = true;

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
    if (!enabled) {
      return ChassisSpeeds.fromFieldRelativeSpeeds(
          xFeedforward, yFeedforward, 0.0, currentPose.getRotation());
    }

    double xFeedback = xController.calculate(currentPose.getX(), targetState.pose.getX());
    double yFeedback = yController.calculate(currentPose.getY(), targetState.pose.getY());
    double rotationFeedback =
        rotationController.calculate(
            currentPose.getRotation().getRadians(), targetState.pose.getRotation().getRadians());
    double rotationFeedforward = targetState.fieldSpeeds.omegaRadiansPerSecond;

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        xFeedforward + xFeedback,
        yFeedforward + yFeedback,
        rotationFeedforward + rotationFeedback,
        currentPose.getRotation());
  }

  @Override
  public void reset(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
    xController.reset();
    yController.reset();
    rotationController.reset();
  }

  @Override
  public boolean isHolonomic() {
    return true;
  }
}
