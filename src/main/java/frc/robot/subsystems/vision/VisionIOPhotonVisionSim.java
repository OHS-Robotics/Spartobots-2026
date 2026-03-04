// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
  private static VisionSystemSim visionSim;

  private final Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim cameraSim;

  /**
   * Creates a new VisionIOPhotonVisionSim.
   *
   * @param name The name of the camera.
   * @param poseSupplier Supplier for the robot pose to use in simulation.
   */
  public VisionIOPhotonVisionSim(
      String name,
      Transform3d robotToCamera,
      int pipelineIndex,
      Supplier<Pose2d> poseSupplier,
      VisionConstants.CameraSimConfig simConfig) {
    super(name, robotToCamera, pipelineIndex);
    this.poseSupplier = poseSupplier;

    // Initialize vision sim
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(aprilTagLayout);
    }

    // Add sim camera
    var cameraProperties =
        new SimCameraProperties()
            .setCalibration(simConfig.width(), simConfig.height(), simConfig.diagonalFov())
            .setCalibError(
                simConfig.calibrationAverageErrorPx(), simConfig.calibrationErrorStdDevPx())
            .setFPS(simConfig.fps())
            .setExposureTimeMs(simConfig.exposureMs())
            .setAvgLatencyMs(simConfig.averageLatencyMs())
            .setLatencyStdDevMs(simConfig.latencyStdDevMs());
    cameraSim =
        new PhotonCameraSim(
            camera, cameraProperties, 0.0, simConfig.maxSightRangeMeters(), aprilTagLayout);
    visionSim.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    visionSim.update(poseSupplier.get());
    super.updateInputs(inputs);
  }
}
