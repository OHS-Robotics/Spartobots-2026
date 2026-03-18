package frc.robot.targeting;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;
import java.util.function.DoubleSupplier;

public class FieldTargetingService {
  private static final Pose2d blueOutpostOpeningShotPose =
      new Pose2d(2.85, 2.1, Rotation2d.fromDegrees(-130.0));
  private static final Pose2d redOutpostOpeningShotPose =
      new Pose2d(
          Constants.fieldLength - 2.85, Constants.fieldWidth - 2.1, Rotation2d.fromDegrees(50.0));
  private static final Pose2d blueLadderAlignPose =
      new Pose2d(1.25, Constants.fieldWidth - 0.75, Rotation2d.kZero);
  private static final Pose2d redLadderAlignPose =
      new Pose2d(Constants.fieldLength - 1.25, 0.75, Rotation2d.fromDegrees(180.0));
  private static final Pose2d blueDepotAlignPose =
      new Pose2d(Constants.blueLine - 0.6, Constants.midLineY, Rotation2d.kZero);
  private static final Pose2d redDepotAlignPose =
      new Pose2d(Constants.redLine + 0.6, Constants.midLineY, Rotation2d.fromDegrees(180.0));

  private final Drive drive;

  public FieldTargetingService(Drive drive) {
    this.drive = drive;
  }

  public Command autoDriveUnderTrenchCommand() {
    return drive.autoDriveUnderTrenchCommand();
  }

  public Command driveToOutpostCommand() {
    return drive.driveToOutpostCommand();
  }

  public Command alignToOutpost(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return drive.alignToOutpost(xSupplier, ySupplier);
  }

  public Command alignToLadderCommand() {
    return drive.alignToPose(getAlliancePose(blueLadderAlignPose, redLadderAlignPose));
  }

  public Command parkAtLadderL1Command() {
    return alignToLadderCommand();
  }

  public Command alignToDepotCommand() {
    return drive.alignToPose(selectDepotAlignPose(DriverStation.getAlliance()));
  }

  public Pose2d getOutpostStartPose() {
    return selectCompetitionAutoStartPose(DriverStation.getAlliance(), drive.getPose());
  }

  public Pose2d getOpeningShotPose() {
    return selectOpeningShotPose(DriverStation.getAlliance());
  }

  public Command driveToOpeningShotCommand() {
    return drive.pathfindToTranslation(getOpeningShotPose().getTranslation());
  }

  static Pose2d selectLadderAlignPose(Optional<Alliance> alliance) {
    return getAlliancePose(alliance, blueLadderAlignPose, redLadderAlignPose);
  }

  static Pose2d selectDepotAlignPose(Optional<Alliance> alliance) {
    return getAlliancePose(alliance, blueDepotAlignPose, redDepotAlignPose);
  }

  static Pose2d selectOutpostStartPose(Optional<Alliance> alliance) {
    return getAlliancePose(alliance, Constants.blueOutpost, Constants.redOutpost);
  }

  static Pose2d selectCompetitionAutoStartPose(Optional<Alliance> alliance, Pose2d currentPose) {
    Pose2d nominalStartPose =
        getAlliancePose(
            alliance, Constants.competitionAutoBlueStart, Constants.competitionAutoRedStart);
    double yOnStartingLine = MathUtil.clamp(currentPose.getY(), 0.0, Constants.fieldWidth);
    return new Pose2d(nominalStartPose.getX(), yOnStartingLine, nominalStartPose.getRotation());
  }

  static Pose2d selectOpeningShotPose(Optional<Alliance> alliance) {
    return getAlliancePose(alliance, blueOutpostOpeningShotPose, redOutpostOpeningShotPose);
  }

  private static Pose2d getAlliancePose(
      Optional<Alliance> alliance, Pose2d bluePose, Pose2d redPose) {
    return alliance.orElse(Alliance.Blue) == Alliance.Red ? redPose : bluePose;
  }

  private Pose2d getAlliancePose(Pose2d bluePose, Pose2d redPose) {
    return getAlliancePose(DriverStation.getAlliance(), bluePose, redPose);
  }
}
