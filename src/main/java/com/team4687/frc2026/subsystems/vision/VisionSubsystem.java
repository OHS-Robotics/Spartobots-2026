package com.team4687.frc2026.subsystems.vision;

import com.team4687.frc2026.Constants;
import com.team4687.frc2026.subsystems.SwerveSubsystem;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class VisionSubsystem extends SubsystemBase {
  private final PhotonCamera camera = new PhotonCamera(Constants.VisionConstants.CAMERA_NAME);
  private final PhotonPoseEstimator photonEstimator;
  private final SwerveSubsystem swerveSubsystem;
  private final Matrix<N3, N1> visionStdDevs = Constants.VisionConstants.VISION_MEASUREMENT_STD_DEVS;
  private Optional<EstimatedRobotPose> latestEstimate = Optional.empty();
  private Alliance lastAlliance;

  public VisionSubsystem(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;

    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(Constants.VisionConstants.FIELD);
    fieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

    photonEstimator =
        new PhotonPoseEstimator(
            fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            Constants.VisionConstants.ROBOT_TO_CAMERA);
    photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    this.swerveSubsystem.swerveDrive.setVisionMeasurementStdDevs(visionStdDevs);
  }

  @Override
  public void periodic() {
    updateFieldOrigin();

    latestEstimate = photonEstimator.update(camera.getLatestResult());
    latestEstimate.ifPresent(
        estimate ->
            swerveSubsystem.swerveDrive.addVisionMeasurement(
                estimate.estimatedPose.toPose2d(), estimate.timestampSeconds));
  }

  public Optional<EstimatedRobotPose> getLatestEstimate() {
    return latestEstimate;
  }

  private void updateFieldOrigin() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty() || alliance.get() == lastAlliance) {
      return;
    }

    lastAlliance = alliance.get();
    OriginPosition origin =
        lastAlliance == Alliance.Red
            ? OriginPosition.kRedAllianceWallRightSide
            : OriginPosition.kBlueAllianceWallRightSide;
    photonEstimator.getFieldTags().setOrigin(origin);
  }
}
