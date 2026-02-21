package com.team4687.frc2026.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.team4687.frc2026.Constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;

public class VisionSubsystem extends SubsystemBase {
    PhotonCamera[] cameras;
    private PhotonPoseEstimator[] poseEstimators;

    public Matrix<N3, N1> stdDevs;

    boolean didWarn = false;

    public VisionSubsystem() {
        cameras = new PhotonCamera[Constants.cameraNames.length];
        poseEstimators = new PhotonPoseEstimator[Constants.cameraNames.length];

        for (int i = 0; i < Constants.cameraNames.length; i++) {
            cameras[i] = new PhotonCamera(Constants.cameraNames[i]);
            cameras[i].setPipelineIndex(0);
            poseEstimators[i] = new PhotonPoseEstimator(Constants.FIELD_LAYOUT, Constants.cameraPositions[i]);
        }
    }

    public void updateStdDevs(Optional<EstimatedRobotPose> currentPose, List<PhotonTrackedTarget> targets, PhotonPoseEstimator poseEstimator) {
        /*
         * Get the standard deviations for pipeline results.
         * I'm still not 100% sure what this is *supposed* to be measuring,
         * but the PhotonVision documentation shows it measuring the average
         * distance between each target(apriltag) and the current estimated pose.
         * At some point I think it might be interesting to test what
         * happens if you measure the average distance between each
         * result and the center point of all of the results.
         * This might not even work, but it could be interesting.
         * Essentially copied from the PhotonLib example
         */

        double averageDistance = 0.0;
        int    measuredTags    = 0;

        if (currentPose.isEmpty()) {
            stdDevs = VecBuilder.fill(4, 4, 8); // todo: choose good values for these
            return;
        }

        for (var target : targets) {
            Optional<Pose3d> pose = poseEstimator.getFieldTags().getTagPose(target.getFiducialId());
            if (pose.isPresent()) continue;
            measuredTags++;
            averageDistance += pose.get().toPose2d().getTranslation().getDistance(
                currentPose.get().estimatedPose.toPose2d().getTranslation()
            );

        }

        if (measuredTags == 0) {
            stdDevs = VecBuilder.fill(4, 4, 8);
        }
        else {
            averageDistance /= measuredTags;

            if (measuredTags > 1) stdDevs = VecBuilder.fill(0.5, 0.5, 1); // more magic numbers to be replaced later

            if (measuredTags == 1 && averageDistance > 4) VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            else stdDevs = stdDevs.times(1 + (averageDistance * averageDistance / 30));
        }
    }

    public void updatePoseEstimate(SwerveDrive swerveDrive) {
        if (!cameras[0].isConnected()) {
            if (!didWarn) {
                didWarn = true;
                System.out.println("*\nCAMERA DISABLED\n*");
            }
            return;
        }

        for (int i = 0; i < cameras.length; i++) {
            PhotonCamera camera = cameras[i];
            PhotonPoseEstimator poseEstimator = poseEstimators[i];

            List<PhotonPipelineResult> results = camera.getAllUnreadResults();
            Optional<EstimatedRobotPose> visionEstimatedPose = Optional.empty();


            for (var result : results) {
                if (result.hasTargets()) System.out.println("Result has targets");
                visionEstimatedPose = poseEstimator.estimateCoprocMultiTagPose(result);
                if (visionEstimatedPose.isEmpty()) visionEstimatedPose = poseEstimator.estimateLowestAmbiguityPose(result);

                if (visionEstimatedPose.isPresent()) System.out.printf("Vision pose: %f %f", visionEstimatedPose.get().estimatedPose.getX(), visionEstimatedPose.get().estimatedPose.getY());

                updateStdDevs(visionEstimatedPose, result.getTargets(), poseEstimator);
                // todo: update standard deviations
                visionEstimatedPose.ifPresent(
                    est -> {
                        swerveDrive.setVisionMeasurementStdDevs(stdDevs);
                        swerveDrive.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds);
                });
            }
        }
    }
}
