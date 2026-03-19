package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.Optional;
import org.junit.jupiter.api.Test;

class TunableHolonomicPathControllerTest {
  private static final double epsilon = 1e-9;

  @Test
  void calculateCombinesFieldRelativeFeedforwardAndFeedback() {
    TunableHolonomicPathController controller =
        new TunableHolonomicPathController(
            new PIDConstants(2.0, 0.0, 0.0), new PIDConstants(3.0), 0.02);
    PathPlannerTrajectoryState targetState =
        createState(
            new Pose2d(2.0, -1.0, Rotation2d.fromDegrees(90.0)), new ChassisSpeeds(1.0, -0.5, 0.2));
    Pose2d currentPose = new Pose2d(1.5, -1.5, Rotation2d.fromDegrees(60.0));

    ChassisSpeeds speeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);

    ChassisSpeeds expected =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            2.0, 0.5, 0.2 + (Math.PI / 6.0) * 3.0, currentPose.getRotation());
    assertSpeedsEqual(expected, speeds);
  }

  @Test
  void rotationOverrideReplacesTrajectoryRotationTarget() {
    TunableHolonomicPathController controller =
        new TunableHolonomicPathController(new PIDConstants(0.0), new PIDConstants(2.0), 0.02);
    controller.setRotationTargetOverride(() -> Optional.of(Rotation2d.k180deg));
    PathPlannerTrajectoryState targetState =
        createState(
            new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(90.0)), new ChassisSpeeds(0.0, 0.0, 0.0));
    Pose2d currentPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(150.0));

    ChassisSpeeds speeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);

    assertEquals(Math.toRadians(60.0), speeds.omegaRadiansPerSecond, epsilon);
  }

  @Test
  void nullRotationOverrideFallsBackToTrajectoryRotation() {
    TunableHolonomicPathController controller =
        new TunableHolonomicPathController(new PIDConstants(0.0), new PIDConstants(2.0), 0.02);
    controller.setRotationTargetOverride(() -> null);
    PathPlannerTrajectoryState targetState =
        createState(
            new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(90.0)), new ChassisSpeeds(0.0, 0.0, 0.0));
    Pose2d currentPose = new Pose2d(0.0, 0.0, Rotation2d.kZero);

    ChassisSpeeds speeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);

    assertEquals(Math.PI, speeds.omegaRadiansPerSecond, epsilon);
  }

  @Test
  void disabledControllerPreservesFullFeedforward() {
    TunableHolonomicPathController controller =
        new TunableHolonomicPathController(new PIDConstants(5.0), new PIDConstants(7.0), 0.02);
    controller.setEnabled(false);
    PathPlannerTrajectoryState targetState =
        createState(
            new Pose2d(3.0, -2.0, Rotation2d.fromDegrees(120.0)),
            new ChassisSpeeds(1.2, -0.8, 0.6));
    Pose2d currentPose = new Pose2d(0.5, 0.25, Rotation2d.fromDegrees(45.0));

    ChassisSpeeds speeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);

    ChassisSpeeds expected =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            targetState.fieldSpeeds.vxMetersPerSecond,
            targetState.fieldSpeeds.vyMetersPerSecond,
            targetState.fieldSpeeds.omegaRadiansPerSecond,
            currentPose.getRotation());
    assertSpeedsEqual(expected, speeds);
  }

  private static PathPlannerTrajectoryState createState(Pose2d pose, ChassisSpeeds fieldSpeeds) {
    PathPlannerTrajectoryState state = new PathPlannerTrajectoryState();
    state.pose = pose;
    state.fieldSpeeds = fieldSpeeds;
    return state;
  }

  private static void assertSpeedsEqual(ChassisSpeeds expected, ChassisSpeeds actual) {
    assertEquals(expected.vxMetersPerSecond, actual.vxMetersPerSecond, epsilon);
    assertEquals(expected.vyMetersPerSecond, actual.vyMetersPerSecond, epsilon);
    assertEquals(expected.omegaRadiansPerSecond, actual.omegaRadiansPerSecond, epsilon);
  }
}
