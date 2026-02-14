package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import org.littletonrobotics.junction.Logger;

public class AlignToPose extends Command {
  private Drive drive;
  private Pose2d target;
  private PIDController driveToPoseController;

  public AlignToPose(Drive drive, Pose2d target, PIDController driveToPoseController) {
    this.drive = drive;
    this.target = target;
    this.driveToPoseController = driveToPoseController;
  }
  @Override
  public void initialize() {
  }

  @Override
  public boolean isFinished() {
    if (drive.getPose().getTranslation().getDistance(target.getTranslation())
        < DriveConstants.alignmentAcceptableDistance) {
      return false;
    }
    return false;
  }

  @Override
  public void execute() {
    Logger.recordOutput("ojhn lokc", 1);
    DriveCommands.joystickDrive(
        drive,
        () -> driveToPoseController.calculate(drive.getPose().getX(), target.getX()),
        () -> driveToPoseController.calculate(drive.getPose().getY(), target.getY()),
        () ->
            -driveToPoseController.calculate(
                drive.getPose().getRotation().getRadians(), target.getRotation().getRadians()));
  }
}
