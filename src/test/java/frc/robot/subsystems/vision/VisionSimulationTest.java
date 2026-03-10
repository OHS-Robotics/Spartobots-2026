package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.FieldConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.Test;

class VisionSimulationTest {
  @Test
  void visionAcceptsInBoundsObservationsAndRejectsImpossibleOnes() {
    List<Pose2d> acceptedPoses = new ArrayList<>();
    Vision vision =
        new Vision(
            (pose, timestamp, stdDevs) -> acceptedPoses.add(pose),
            new FakeVisionIO(
                new VisionIO.PoseObservation[] {
                  new VisionIO.PoseObservation(
                      1.0,
                      new Pose3d(2.0, 2.0, 0.1, new edu.wpi.first.math.geometry.Rotation3d()),
                      0.1,
                      2,
                      1.5,
                      VisionIO.PoseObservationType.PHOTONVISION),
                  new VisionIO.PoseObservation(
                      1.1,
                      new Pose3d(-0.5, 0.0, 2.0, new edu.wpi.first.math.geometry.Rotation3d()),
                      0.9,
                      1,
                      2.5,
                      VisionIO.PoseObservationType.PHOTONVISION)
                }));

    vision.periodic();

    assertEquals(1, acceptedPoses.size());
    assertEquals(2.0, acceptedPoses.get(0).getX(), 1e-9);
    assertEquals(2.0, acceptedPoses.get(0).getY(), 1e-9);
  }

  @Test
  void photonVisionSimProducesAtLeastOneObservationNearFieldTags() {
    AtomicReference<Pose2d> poseSupplier = new AtomicReference<>(Pose2d.kZero);
    VisionIOPhotonVisionSim io =
        new VisionIOPhotonVisionSim("SimVisionTest", new Transform3d(), poseSupplier::get);
    VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();

    boolean foundObservation = false;
    for (var aprilTag : FieldConstants.APRIL_TAG_LAYOUT.getTags()) {
      Pose3d tagPose = aprilTag.pose;
      for (Translation2d offset :
          List.of(
              new Translation2d(-1.5, 0.0),
              new Translation2d(1.5, 0.0),
              new Translation2d(0.0, -1.5),
              new Translation2d(0.0, 1.5))) {
        double candidateX = tagPose.getX() + offset.getX();
        double candidateY = tagPose.getY() + offset.getY();
        if (candidateX < 0.1
            || candidateX > FieldConstants.FIELD_LENGTH_METERS - 0.1
            || candidateY < 0.1
            || candidateY > FieldConstants.FIELD_WIDTH_METERS - 0.1) {
          continue;
        }

        Rotation2d heading =
            Rotation2d.fromRadians(
                Math.atan2(tagPose.getY() - candidateY, tagPose.getX() - candidateX));
        poseSupplier.set(new Pose2d(candidateX, candidateY, heading));
        for (int i = 0; i < 3; i++) {
          io.updateInputs(inputs);
        }

        if (inputs.poseObservations.length > 0 || inputs.tagIds.length > 0) {
          foundObservation = true;
          break;
        }
      }
      if (foundObservation) {
        break;
      }
    }

    assertTrue(foundObservation, "PhotonVision sim should see at least one field tag");
  }

  private static final class FakeVisionIO implements VisionIO {
    private final PoseObservation[] poseObservations;

    private FakeVisionIO(PoseObservation[] poseObservations) {
      this.poseObservations = poseObservations;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
      inputs.connected = true;
      inputs.poseObservations = poseObservations;
      inputs.tagIds = new int[] {1, 2};
    }
  }
}
