package com.team4687.frc2026.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.team4687.frc2026.Constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;

public class VisionSubsystem extends SubsystemBase {
    private PhotonPoseEstimator poseEstimator;
    // todo: change the name of this camera because it's very long
    private PhotonCamera camera;
    private boolean initialized = false;

    public Matrix<N3, N1> stdDevs;

    public void initVision() {
        poseEstimator = new PhotonPoseEstimator(Constants.FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.cameraPosition);
        camera = new PhotonCamera("Arducam_OV9281_USB_Camera");
        initialized = true;
    }

    public void updateStdDevs(Pose2d currentPose, List<PhotonTrackedTarget> targets) {
        /*
         * Get the standard deviations for pipeline results.
         * I'm still not 100% sure what this is *supposed* to be measuring,
         * but the PhotonVision documentation shows it measuring the average
         * distance between each target(apriltag) and the current estimated pose.
         * At some point I think it might be interesting to test what
         * happens if you measure the average distance between each
         * result and the center point of all of the results.
         * This might not even work, but it could be interesting.
         */

        double averageDistance = 0.0;
        int    measuredTags    = 0;

        for (var target : targets) {
            Optional<Pose3d> pose = poseEstimator.getFieldTags().getTagPose(target.getFiducialId());
        }
    }

    public void updatePoseEstimate(SwerveDrive swerveDrive) {
        if (!initialized) return;
        
        //Optional<Pose2d> visionEstimatedPose = poseEstimator.

        Optional<EstimatedRobotPose> visionEst = Optional.empty();

        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        for (var change : results) {
            visionEst = poseEstimator.update(change);


            visionEst.ifPresent(
                est -> {
                    
                    swerveDrive.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds);
                    
            });
        }
    }
}
