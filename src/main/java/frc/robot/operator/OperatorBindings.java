package frc.robot.operator;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.gamepiece.intake.Intake;
import frc.robot.subsystems.gamepiece.shooter.Shooter;
import frc.robot.superstructure.gamepiece.GamePieceCoordinator;
import frc.robot.targeting.FieldTargetingService;
import frc.robot.targeting.HubTargetingService;
import java.util.function.DoubleSupplier;

public class OperatorBindings {
  private static final double manualHoodStepDegrees = 0.35;
  private static final boolean enableMechanismBringupBindings = false;
  private static final double intakePivotManualSpeed = 0.25;
  private static final double hopperExtensionBringupSpeed = 0.25;
  private static final int topLeftPaddleButton = 7;
  private static final int topRightPaddleButton = 8;
  private static final int bottomLeftPaddleButton = 11;

  private final CommandXboxController driverController;
  private final CommandXboxController operatorController;
  private final Drive drive;
  private final Intake intake;
  private final Shooter shooter;
  private final GamePieceCoordinator gamePieceCoordinator;
  private final HubTargetingService hubTargetingService;
  private final FieldTargetingService fieldTargetingService;
  private final AutoAssistController autoAssistController;

  public OperatorBindings(
      CommandXboxController driverController,
      CommandXboxController operatorController,
      Drive drive,
      Intake intake,
      Shooter shooter,
      GamePieceCoordinator gamePieceCoordinator,
      HubTargetingService hubTargetingService,
      FieldTargetingService fieldTargetingService,
      AutoAssistController autoAssistController) {
    this.driverController = driverController;
    this.operatorController = operatorController;
    this.drive = drive;
    this.intake = intake;
    this.shooter = shooter;
    this.gamePieceCoordinator = gamePieceCoordinator;
    this.hubTargetingService = hubTargetingService;
    this.fieldTargetingService = fieldTargetingService;
    this.autoAssistController = autoAssistController;
  }

  public void configure() {
    configureDriverBindings();
    configureOperatorBindings();
  }

  private void configureDriverBindings() {
    DoubleSupplier driverForwardSupplier = driverController::getLeftY;
    DoubleSupplier driverStrafeSupplier = driverController::getLeftX;

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            driverForwardSupplier,
            driverStrafeSupplier,
            () -> -driverController.getRightX(),
            () -> false));

    driverController
        .rightStick()
        .whileTrue(
            hubTargetingService.alignToHub(
                driverForwardSupplier, driverStrafeSupplier, gamePieceCoordinator));

    new Trigger(() -> driverController.getRightTriggerAxis() > 0.02)
        .whileTrue(
            gamePieceCoordinator.runShooterDemandWhileHeldCommand(
                driverController::getRightTriggerAxis));
    new Trigger(() -> driverController.getLeftTriggerAxis() > 0.02)
        .whileTrue(
            gamePieceCoordinator.runManualFeedAndIndexersWhileHeldCommand(
                driverController::getLeftTriggerAxis));

    driverController.leftBumper().onTrue(Commands.runOnce(autoAssistController::cancel));
    driverController
        .rightBumper()
        .onTrue(
            autoAssistController.scheduleAction(
                "AutoAssist/DriveUnderTrench", fieldTargetingService::autoDriveUnderTrenchCommand));
    driverController
        .leftStick()
        .whileTrue(Commands.run(drive::stopWithX, drive));

    if (!enableMechanismBringupBindings) {
      new Trigger(() -> driverController.getHID().getRawButton(topLeftPaddleButton))
          .onTrue(
              autoAssistController.scheduleAction(
                  "AutoAssist/ParkAtLadderL1", fieldTargetingService::parkAtLadderL1Command));
      new Trigger(() -> driverController.getHID().getRawButton(topRightPaddleButton))
          .onTrue(
              autoAssistController.scheduleAction(
                  "AutoAssist/DriveToOutpost", fieldTargetingService::driveToOutpostCommand));
      new Trigger(() -> driverController.getHID().getRawButton(bottomLeftPaddleButton))
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
    driverController.y().whileTrue(gamePieceCoordinator.basicCollectWhileHeldCommand(true));
    driverController.x().whileTrue(gamePieceCoordinator.basicCollectWhileHeldCommand(false));
    driverController.a().whileTrue(gamePieceCoordinator.basicReverseWhileHeldCommand());
    driverController.b().onTrue(Commands.runOnce(gamePieceCoordinator::stopGamePieceFlow));

    driverController
        .povUp()
        .whileTrue(Commands.run(() -> shooter.adjustHoodSetpointDegrees(manualHoodStepDegrees)));
    driverController
        .povDown()
        .whileTrue(Commands.run(() -> shooter.adjustHoodSetpointDegrees(-manualHoodStepDegrees)));

    driverController
        .povLeft()
        .whileTrue(intake.manualIntakePivotWhileHeldCommand(-intakePivotManualSpeed));
    driverController
        .povRight()
        .whileTrue(intake.manualIntakePivotWhileHeldCommand(intakePivotManualSpeed));

    if (enableMechanismBringupBindings) {
      driverController
          .back()
          .whileTrue(
              gamePieceCoordinator.runHopperExtensionWhileHeldCommand(
                  -hopperExtensionBringupSpeed));
      driverController
          .start()
          .whileTrue(
              gamePieceCoordinator.runHopperExtensionWhileHeldCommand(hopperExtensionBringupSpeed));
    } else {
      driverController.back().onTrue(Commands.runOnce(gamePieceCoordinator::stopGamePieceFlow));
      driverController.start().onTrue(intake.calibrateIntakePivotToHardStopsCommand());
    }
  }
}
