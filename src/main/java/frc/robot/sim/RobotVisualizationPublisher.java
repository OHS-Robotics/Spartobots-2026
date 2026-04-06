package frc.robot.sim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.gamepiece.intake.Intake;
import frc.robot.subsystems.gamepiece.shooter.Shooter;
import frc.robot.subsystems.gamepiece.shooter.ShooterConstants;
import frc.robot.util.NetworkTablesUtil;
import org.littletonrobotics.junction.Logger;

public class RobotVisualizationPublisher {
  private static final int componentIndexIntakePivot = 0;
  private static final int componentIndexShooterHood = 1;
  private static final int robotComponentCount = 2;
  private static final Translation3d intakePivotOriginOnRobot =
      new Translation3d(0.305, 0.0, 0.215);
  private static final Rotation2d intakePivotRetractedAngle = Rotation2d.fromDegrees(0.0);
  private static final Rotation2d intakePivotExtendedAngle = Rotation2d.fromDegrees(-82.0);
  private static final Translation3d shooterHoodPivotOnRobot = new Translation3d(-0.23, 0.0, 0.56);
  private static final Rotation2d shooterHoodModelPitchOffset = Rotation2d.kZero;

  private final Intake intake;
  private final Shooter shooter;

  public RobotVisualizationPublisher(Intake intake, Shooter shooter) {
    this.intake = intake;
    this.shooter = shooter;
  }

  public void publish() {
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Simulation/RobotModel/Components"),
        getRobotRelativeComponentPoses());
  }

  private Pose3d[] getRobotRelativeComponentPoses() {
    Pose3d[] componentPoses = new Pose3d[robotComponentCount];

    double intakePivotNormalized = intake.getIntakePivotMeasuredPositionNormalized();
    Rotation2d intakePivotPitch =
        Rotation2d.fromDegrees(
            MathUtil.interpolate(
                intakePivotRetractedAngle.getDegrees(),
                intakePivotExtendedAngle.getDegrees(),
                intakePivotNormalized));
    componentPoses[componentIndexIntakePivot] =
        new Pose3d(
            intakePivotOriginOnRobot, new Rotation3d(0.0, intakePivotPitch.getRadians(), 0.0));

    Rotation2d hoodPitch =
        shooter
            .getMeasuredHoodAngle()
            .minus(ShooterConstants.minHoodAngleFromFloor)
            .plus(shooterHoodModelPitchOffset);
    componentPoses[componentIndexShooterHood] =
        new Pose3d(shooterHoodPivotOnRobot, new Rotation3d(0.0, hoodPitch.getRadians(), 0.0));

    return componentPoses;
  }
}
