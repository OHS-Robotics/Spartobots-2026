package frc.robot.operator;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.gamepiece.intake.Intake;
import frc.robot.subsystems.gamepiece.shooter.Shooter;
import frc.robot.superstructure.gamepiece.GamePieceCoordinator;
import frc.robot.targeting.FieldTargetingService;
import frc.robot.targeting.HubTargetingService;

public class OperatorBindings {
  private static final double manualHoodStepDegrees = 0.35;
  private static final boolean enableDriverStationCharacterization = true;
  private static final boolean enableMechanismBringupBindings = false;
  private static final double intakePivotManualSpeed = 0.25;
  private static final double hopperExtensionBringupSpeed = 0.25;
  private static final int topLeftPaddleButton = 7;
  private static final int topRightPaddleButton = 8;
  private static final int bottomLeftPaddleButton = 11;

  private final CommandGenericHID driverHid;
  private final CommandGenericHID operatorHid;
  private final Drive drive;
  private final Intake intake;
  private final Shooter shooter;
  private final GamePieceCoordinator gamePieceCoordinator;
  private final HubTargetingService hubTargetingService;
  private final FieldTargetingService fieldTargetingService;
  private final AutoAssistController autoAssistController;

  public OperatorBindings(
      CommandGenericHID driverHid,
      CommandGenericHID operatorHid,
      Drive drive,
      Intake intake,
      Shooter shooter,
      GamePieceCoordinator gamePieceCoordinator,
      HubTargetingService hubTargetingService,
      FieldTargetingService fieldTargetingService,
      AutoAssistController autoAssistController) {
    this.driverHid = driverHid;
    this.operatorHid = operatorHid;
    this.drive = drive;
    this.intake = intake;
    this.shooter = shooter;
    this.gamePieceCoordinator = gamePieceCoordinator;
    this.hubTargetingService = hubTargetingService;
    this.fieldTargetingService = fieldTargetingService;
    this.autoAssistController = autoAssistController;
  }

  public void configure() {
    if (enableDriverStationCharacterization) {
      return;
    }

    configureDriverBindings();
    configureOperatorBindings();
  }

  private void configureDriverBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> driverHid.getRawAxis(1),
            () -> driverHid.getRawAxis(0),
            () -> -driverHid.getRawAxis(4),
            () -> false));

    driverHid
        .button(10)
        .whileTrue(
            hubTargetingService.alignToHub(
                () -> -driverHid.getRawAxis(1),
                () -> -driverHid.getRawAxis(0),
                gamePieceCoordinator));

    new Trigger(() -> driverHid.getRawAxis(3) > 0.02)
        .whileTrue(
            gamePieceCoordinator.runShooterDemandWhileHeldCommand(() -> driverHid.getRawAxis(3)));
    new Trigger(() -> driverHid.getRawAxis(2) > 0.02)
        .whileTrue(
            gamePieceCoordinator.runManualFeedAndIndexersWhileHeldCommand(
                () -> driverHid.getRawAxis(2)));

    driverHid.button(5).onTrue(Commands.runOnce(autoAssistController::cancel));
    driverHid
        .button(6)
        .onTrue(
            autoAssistController.scheduleAction(
                "AutoAssist/DriveUnderTrench", fieldTargetingService::autoDriveUnderTrenchCommand));
    driverHid
        .button(9)
        .onTrue(
            autoAssistController.scheduleAction(
                "AutoAssist/ParkAtLadderL1", fieldTargetingService::parkAtLadderL1Command));

    if (!enableMechanismBringupBindings) {
      new Trigger(() -> driverHid.getHID().getRawButton(topLeftPaddleButton))
          .onTrue(
              autoAssistController.scheduleAction(
                  "AutoAssist/ParkAtLadderL1", fieldTargetingService::parkAtLadderL1Command));
      new Trigger(() -> driverHid.getHID().getRawButton(topRightPaddleButton))
          .onTrue(
              autoAssistController.scheduleAction(
                  "AutoAssist/DriveToOutpost", fieldTargetingService::driveToOutpostCommand));
      new Trigger(() -> driverHid.getHID().getRawButton(bottomLeftPaddleButton))
          .onTrue(
              autoAssistController.scheduleAction(
                  "AutoAssist/AlignToDepot", fieldTargetingService::alignToDepotCommand));
    }

    if (Constants.currentMode == Constants.Mode.SIM) {
      // Reserved for simulation-only bindings.
    }
  }

  private void configureOperatorBindings() {
    // Consolidated single-controller layout: operator actions live on the driver controller.
    driverHid.button(4).whileTrue(gamePieceCoordinator.basicCollectWhileHeldCommand(true));
    driverHid.button(3).whileTrue(gamePieceCoordinator.basicCollectWhileHeldCommand(false));
    driverHid.button(1).whileTrue(gamePieceCoordinator.basicReverseWhileHeldCommand());
    driverHid.button(2).onTrue(Commands.runOnce(gamePieceCoordinator::stopGamePieceFlow));

    driverHid
        .povUp()
        .whileTrue(Commands.run(() -> shooter.adjustHoodSetpointDegrees(manualHoodStepDegrees)));
    driverHid
        .povDown()
        .whileTrue(Commands.run(() -> shooter.adjustHoodSetpointDegrees(-manualHoodStepDegrees)));

    driverHid
        .povLeft()
        .whileTrue(intake.manualIntakePivotWhileHeldCommand(-intakePivotManualSpeed));
    driverHid
        .povRight()
        .whileTrue(intake.manualIntakePivotWhileHeldCommand(intakePivotManualSpeed));

    if (enableMechanismBringupBindings) {
      driverHid
          .button(7)
          .whileTrue(
              gamePieceCoordinator.runHopperExtensionWhileHeldCommand(
                  -hopperExtensionBringupSpeed));
      driverHid
          .button(8)
          .whileTrue(
              gamePieceCoordinator.runHopperExtensionWhileHeldCommand(hopperExtensionBringupSpeed));
    } else {
      driverHid.button(7).onTrue(Commands.runOnce(gamePieceCoordinator::stopGamePieceFlow));
      driverHid.button(8).onTrue(intake.calibrateIntakePivotToHardStopsCommand());
    }
  }
}
