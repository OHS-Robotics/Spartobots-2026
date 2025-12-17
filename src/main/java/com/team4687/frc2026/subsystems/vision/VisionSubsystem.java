package com.team4687.frc2026.subsystems.vision;

import com.team4687.frc2026.Constants;
import com.team4687.frc2026.subsystems.SwerveSubsystem;
import com.team4687.frc2026.subsystems.vision.estimation.PhotonVisionPoseEstimator;
import com.team4687.frc2026.subsystems.vision.estimation.VisionPoseEstimator;
import com.team4687.frc2026.subsystems.vision.fusion.VisionFusionConfig;
import com.team4687.frc2026.subsystems.vision.fusion.VisionSensorFusion;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;

/**
 * Vision-facing subsystem.
 *
 * <p>Implementation detail note: this class intentionally stays small. PhotonVision estimation and
 * fusion/gating logic live in separate modules so we can swap vision implementations later.
 */
public class VisionSubsystem extends SubsystemBase {
  private final SwerveSubsystem swerveSubsystem;
  private final VisionPoseEstimator poseEstimator;
  private final VisionSensorFusion sensorFusion;

  private Optional<VisionSensorFusion.VisionMeasurement> latestAcceptedMeasurement = Optional.empty();

  public VisionSubsystem(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;

    ADIS16470_IMU rioImu = new ADIS16470_IMU();
    rioImu.setYawAxis(ADIS16470_IMU.IMUAxis.kZ);

    poseEstimator =
        new PhotonVisionPoseEstimator(
            Constants.VisionConstants.CAMERA_NAME,
            Constants.VisionConstants.ROBOT_TO_CAMERA,
            Constants.VisionConstants.FIELD);

    sensorFusion =
        new VisionSensorFusion(
            () -> Rotation2d.fromDegrees(rioImu.getAngle()),
            Constants.VisionConstants.VISION_MEASUREMENT_STD_DEVS,
            new VisionFusionConfig(
                Constants.VisionConstants.MAX_POSE_AMBIGUITY,
                Constants.VisionConstants.MAX_TRANSLATION_JUMP_METERS,
                Constants.VisionConstants.MAX_HEADING_ERROR_RADIANS,
                Constants.VisionConstants.STD_DEV_SCALE_SINGLE_TAG));

    this.swerveSubsystem.swerveDrive.setVisionMeasurementStdDevs(
        Constants.VisionConstants.VISION_MEASUREMENT_STD_DEVS);
  }

  @Override
  public void periodic() {
    latestAcceptedMeasurement =
        poseEstimator
            .estimate(swerveSubsystem.getPose())
            .flatMap(estimate -> sensorFusion.fuse(estimate, swerveSubsystem.getPose()));

    latestAcceptedMeasurement.ifPresent(
        measurement ->
            swerveSubsystem.swerveDrive.addVisionMeasurement(
                measurement.pose(), measurement.timestampSeconds(), measurement.stdDevs()));
  }

  public Optional<VisionSensorFusion.VisionMeasurement> getLatestAcceptedMeasurement() {
    return latestAcceptedMeasurement;
  }
}
