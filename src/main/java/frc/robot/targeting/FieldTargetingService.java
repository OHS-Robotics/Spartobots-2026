package frc.robot.targeting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
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
    return drive.alignToPose(getAlliancePose(blueDepotAlignPose, redDepotAlignPose));
  }

  public Pose2d getOutpostStartPose() {
    return getAlliancePose(Constants.blueOutpost, Constants.redOutpost);
  }

  public Pose2d getOpeningShotPose() {
    return getAlliancePose(blueOutpostOpeningShotPose, redOutpostOpeningShotPose);
  }

  private Pose2d getAlliancePose(Pose2d bluePose, Pose2d redPose) {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? redPose : bluePose;
  }
}
