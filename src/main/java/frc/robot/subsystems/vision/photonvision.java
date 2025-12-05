package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class photonvision {
    
    // Change this to match the name of your camera
    private PhotonCamera camera = new PhotonCamera("Arducam_OV9281_USB_Camera");
    private PhotonCamera camera2 = new PhotonCamera("Intel(R)_RealSense(TM)_Depth_Camera_455__RGB");

}
