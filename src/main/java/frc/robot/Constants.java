// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants{
    public static class OperatorConstants {
      public static final int kDriverControllerPort = 0;
      public static final double deadband = 0.7;
    }
  
    public static final double MAX_SPEED = Units.feetToMeters(3.0);
  
    public static class Vision {
      public static final String kCameraName = "Arducam_OV9281_USB_Camera";
      // Cam mounted facing forward, half a meter forward of center, half a meter up from center,
      // pitched upward.
    private static final double camPitch = Units.degreesToRadians(30.0); // Angle of camera
      public static final Transform3d kRobotToCam =
            new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, -camPitch, 0)); // Location on robot

      // The layout of the AprilTags on the field
      public static final AprilTagFieldLayout kTagLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

      // The standard deviations of our vision estimated poses, which affect correction rate
      // (Fake values. Experiment and determine estimation noise on an actual robot.)
      public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
      public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }

    public static class Operator
  {
    // Joystick Deadband
    public static final double deadband = 0.05;
    public static final double deadbandLeftY = 0.05;
    public static final double deadbandRightX = 0.05;
    public static final double turnConstant = 6;
    public static final double scaleTranslationHighGear = 0.8;
    public static final double scaleTranslationLowGear = 0.2;
    public static final double scaleRotationHighGear = 0.8;
    public static final double scaleRotationLowGear = 0.2;
    public static final boolean useKeyboardInSim = true; // Whether to expect keyboard or controller controls in sim

    public static final double nudgeSpeed_MetersPerSec = 0.5;
    public static final double nudgeDistForward_Meters = Units.inchesToMeters(2);
    public static final double nudgeDistBack_Meters = Units.inchesToMeters(2);
    public static final double nudgeDistLeft_Meters = Units.inchesToMeters(2);
    public static final double nudgeDistRight_Meters = Units.inchesToMeters(2);

    public static final boolean useJoystick = false; //false = use xbox

    public static final double expoCurveExponentTranslation = 2;
    public static final double expoCurveExponentRotation = 2;
  }
}