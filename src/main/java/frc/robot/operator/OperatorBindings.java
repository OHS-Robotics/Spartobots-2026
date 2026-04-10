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
import frc.robot.targeting.HubTargetingService;
import java.util.function.DoubleSupplier;

public class OperatorBindings {
  private static final double manualHoodStepDegrees = 0.35;
  private static final double manualOverrideHoodAngleDegrees = 58.0;
  private static final double manualOverrideShooterSpeedRpm = 300.0;
  private static final double controllerActivityDeadband = 0.05;
  private static final double intakePivotManualSpeed = 0.25;

  private final CommandXboxController driverController;
  private final CommandXboxController operatorController;
  private final Drive drive;
  private final Intake intake;
  private final Shooter shooter;
  private final GamePieceCoordinator gamePieceCoordinator;
  private final HubTargetingService hubTargetingService;
  private final AutoAssistController autoAssistController;

  public OperatorBindings(
      CommandXboxController driverController,
      CommandXboxController operatorController,
      Drive drive,
      Intake intake,
      Shooter shooter,
      GamePieceCoordinator gamePieceCoordinator,
      HubTargetingService hubTargetingService,
      AutoAssistController autoAssistController) {
    this.driverController = driverController;
    this.operatorController = operatorController;
    this.drive = drive;
    this.intake = intake;
    this.shooter = shooter;
    this.gamePieceCoordinator = gamePieceCoordinator;
    this.hubTargetingService = hubTargetingService;
    this.autoAssistController = autoAssistController;
  }

  public void configure() {
    configureDriverBindings();
    configureOperatorBindings();
  }

  private void configureDriverBindings() {
    // Match the drivetrain convention used by DriveCommands: +X forward, +Y left.
    DoubleSupplier driverForwardSupplier =
        selectControllerAxis(driverController::getLeftY, operatorController::getLeftY, true);
    DoubleSupplier driverStrafeSupplier =
        selectControllerAxis(driverController::getLeftX, operatorController::getLeftX, true);
    DoubleSupplier driverTurnSupplier =
        selectControllerAxis(driverController::getRightX, operatorController::getRightX, true);

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, driverForwardSupplier, driverStrafeSupplier, driverTurnSupplier, () -> false));

    eitherController(CommandXboxController::rightStick)
        .whileTrue(
            hubTargetingService.alignToHub(
                driverForwardSupplier, driverStrafeSupplier, gamePieceCoordinator));

    new Trigger(
            () ->
                Math.max(
                        driverController.getRightTriggerAxis(),
                        operatorController.getRightTriggerAxis())
                    > 0.02)
        .whileTrue(
            gamePieceCoordinator.runShooterDemandWhileHeldCommand(
                () ->
                    Math.max(
                        driverController.getRightTriggerAxis(),
                        operatorController.getRightTriggerAxis())));
    new Trigger(
            () ->
                Math.max(
                        driverController.getLeftTriggerAxis(),
                        operatorController.getLeftTriggerAxis())
                    > 0.02)
        .whileTrue(
            gamePieceCoordinator.runManualFeedAndIndexersWhileHeldCommand(
                () ->
                    Math.max(
                        driverController.getLeftTriggerAxis(),
                        operatorController.getLeftTriggerAxis())));

    eitherController(CommandXboxController::leftBumper)
        .onTrue(
            Commands.runOnce(
                () -> {
                  autoAssistController.cancel();
                  drive.stop();
                }));
    eitherController(CommandXboxController::leftStick)
        .whileTrue(Commands.run(drive::stopWithX, drive));

    if (Constants.currentMode == Constants.Mode.SIM) {
      // Reserved for simulation-only bindings.
    }
  }

  private void configureOperatorBindings() {
    eitherController(CommandXboxController::a)
        .or(driverController.rightBumper())
        .or(operatorController.rightBumper())
        .toggleOnTrue(gamePieceCoordinator.basicCollectWhileHeldCommand(false));
    eitherController(CommandXboxController::b)
        .whileTrue(gamePieceCoordinator.basicReverseWhileHeldCommand());
    eitherController(CommandXboxController::x)
        .onTrue(
            Commands.runOnce(
                () -> shooter.setManualWheelSpeedRpm(manualOverrideShooterSpeedRpm), shooter));
    eitherController(CommandXboxController::y)
        .onTrue(
            Commands.runOnce(
                () -> shooter.setManualHoodSetpointDegrees(manualOverrideHoodAngleDegrees),
                shooter));

    eitherController(CommandXboxController::povUp)
        .whileTrue(Commands.run(() -> shooter.adjustHoodSetpointDegrees(manualHoodStepDegrees)));
    eitherController(CommandXboxController::povDown)
        .whileTrue(Commands.run(() -> shooter.adjustHoodSetpointDegrees(-manualHoodStepDegrees)));

    eitherController(CommandXboxController::povLeft)
        .whileTrue(intake.manualIntakePivotWhileHeldCommand(-intakePivotManualSpeed));
    eitherController(CommandXboxController::povRight)
        .whileTrue(intake.manualIntakePivotWhileHeldCommand(intakePivotManualSpeed));
  }

  private DoubleSupplier selectControllerAxis(
      DoubleSupplier driverAxis, DoubleSupplier operatorAxis, boolean invertOutput) {
    return () -> {
      double driverValue = driverAxis.getAsDouble();
      double operatorValue = operatorAxis.getAsDouble();
      double selectedValue =
          Math.abs(driverValue) >= Math.abs(operatorValue) ? driverValue : operatorValue;
      if (Math.abs(selectedValue) < controllerActivityDeadband) {
        return 0.0;
      }
      return invertOutput ? -selectedValue : selectedValue;
    };
  }

  private Trigger eitherController(
      java.util.function.Function<CommandXboxController, Trigger> triggerFactory) {
    return triggerFactory.apply(driverController).or(triggerFactory.apply(operatorController));
  }
}
