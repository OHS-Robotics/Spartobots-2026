// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.auto.AutoRoutines;
import frc.robot.game.GameStateSubsystem;
import frc.robot.operator.AutoAssistController;
import frc.robot.operator.OperatorActionPublisher;
import frc.robot.operator.OperatorBindings;
import frc.robot.operator.OperatorDashboard;
import frc.robot.operator.OperatorFeedbackController;
import frc.robot.sim.FieldSimulationManager;
import frc.robot.sim.RobotVisualizationPublisher;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.gamepiece.hopper.Hopper;
import frc.robot.subsystems.gamepiece.hopper.HopperIO;
import frc.robot.subsystems.gamepiece.hopper.HopperIOSim;
import frc.robot.subsystems.gamepiece.hopper.HopperIOSparkMax;
import frc.robot.subsystems.gamepiece.indexers.Indexers;
import frc.robot.subsystems.gamepiece.indexers.IndexersIO;
import frc.robot.subsystems.gamepiece.indexers.IndexersIOSim;
import frc.robot.subsystems.gamepiece.indexers.IndexersIOSparkMax;
import frc.robot.subsystems.gamepiece.intake.Intake;
import frc.robot.subsystems.gamepiece.intake.IntakeIO;
import frc.robot.subsystems.gamepiece.intake.IntakeIOSim;
import frc.robot.subsystems.gamepiece.intake.IntakeIOSparkMax;
import frc.robot.subsystems.gamepiece.shooter.Shooter;
import frc.robot.subsystems.gamepiece.shooter.ShooterIO;
import frc.robot.subsystems.gamepiece.shooter.ShooterIOSim;
import frc.robot.subsystems.gamepiece.shooter.ShooterIOSparkMax;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.superstructure.gamepiece.GamePieceCoordinator;
import frc.robot.targeting.FieldTargetingService;
import frc.robot.targeting.HubTargetingService;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static final int driverControllerPort = 0;
  private static final int operatorControllerPort = 1;

  private final Drive drive;
  private final Shooter shooter;
  private final Intake intake;
  private final Hopper hopper;
  private final Indexers indexers;
  private final GameStateSubsystem gameState;

  @SuppressWarnings("unused")
  private final Vision vision;

  public final CommandXboxController driverController =
      new CommandXboxController(driverControllerPort);
  public final CommandXboxController operatorController =
      new CommandXboxController(operatorControllerPort);

  private final LoggedDashboardChooser<Command> autoChooser;
  private final GamePieceCoordinator gamePieceCoordinator;
  private final HubTargetingService hubTargetingService;
  private final FieldTargetingService fieldTargetingService;
  private final AutoRoutines autoRoutines;
  private final OperatorDashboard operatorDashboard;
  private final OperatorFeedbackController operatorFeedbackController;
  private final AutoAssistController autoAssistController;
  private final RobotVisualizationPublisher robotVisualizationPublisher;
  private final FieldSimulationManager fieldSimulationManager;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    Drive driveLocal;
    Shooter shooterLocal;
    Intake intakeLocal;
    Hopper hopperLocal;
    Indexers indexersLocal;
    Vision visionLocal;
    SwerveDriveSimulation driveSimulationLocal = null;

    switch (Constants.currentMode) {
      case REAL:
        intakeLocal = new Intake(new IntakeIOSparkMax());
        hopperLocal = new Hopper(new HopperIOSparkMax());
        indexersLocal = new Indexers(new IndexersIOSparkMax());
        driveLocal =
            new Drive(
                new GyroIONavX(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        shooterLocal = new Shooter(new ShooterIOSparkMax());
        visionLocal =
            new Vision(
                driveLocal::addVisionMeasurement,
                driveLocal::getPose,
                new VisionIOPhotonVision(
                    VisionConstants.camera0Name,
                    VisionConstants.robotToCamera0,
                    VisionConstants.camera0Pipeline),
                new VisionIOPhotonVision(
                    VisionConstants.camera1Name,
                    VisionConstants.robotToCamera1,
                    VisionConstants.camera1Pipeline));
        break;

      case SIM:
        SimulatedArena.overrideInstance(new Arena2026Rebuilt(false));
        driveSimulationLocal =
            new SwerveDriveSimulation(
                DriveConstants.mapleSimConfig, new Pose2d(3.0, 3.0, Rotation2d.kZero));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulationLocal);
        intakeLocal = new Intake(new IntakeIOSim());
        hopperLocal = new Hopper(new HopperIOSim());
        indexersLocal = new Indexers(new IndexersIOSim());
        driveLocal =
            new Drive(
                new GyroIOSim(driveSimulationLocal.getGyroSimulation()),
                new ModuleIOSim(driveSimulationLocal.getModules()[0]),
                new ModuleIOSim(driveSimulationLocal.getModules()[1]),
                new ModuleIOSim(driveSimulationLocal.getModules()[2]),
                new ModuleIOSim(driveSimulationLocal.getModules()[3]),
                driveSimulationLocal::setSimulationWorldPose);
        shooterLocal = new Shooter(new ShooterIOSim());
        visionLocal =
            new Vision(
                driveLocal::addVisionMeasurement,
                driveSimulationLocal::getSimulatedDriveTrainPose,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name,
                    VisionConstants.robotToCamera0,
                    VisionConstants.camera0Pipeline,
                    driveSimulationLocal::getSimulatedDriveTrainPose,
                    VisionConstants.camera0SimConfig),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera1Name,
                    VisionConstants.robotToCamera1,
                    VisionConstants.camera1Pipeline,
                    driveSimulationLocal::getSimulatedDriveTrainPose,
                    VisionConstants.camera1SimConfig));
        break;

      default:
        intakeLocal = new Intake(new IntakeIO() {});
        hopperLocal = new Hopper(new HopperIO() {});
        indexersLocal = new Indexers(new IndexersIO() {});
        driveLocal =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        shooterLocal = new Shooter(new ShooterIO() {});
        visionLocal =
            new Vision(
                driveLocal::addVisionMeasurement,
                driveLocal::getPose,
                new VisionIOPhotonVision(
                    VisionConstants.camera0Name,
                    VisionConstants.robotToCamera0,
                    VisionConstants.camera0Pipeline),
                new VisionIOPhotonVision(
                    VisionConstants.camera1Name,
                    VisionConstants.robotToCamera1,
                    VisionConstants.camera1Pipeline));
        break;
    }

    drive = driveLocal;
    shooter = shooterLocal;
    intake = intakeLocal;
    hopper = hopperLocal;
    indexers = indexersLocal;
    gameState = new GameStateSubsystem();
    vision = visionLocal;

    gamePieceCoordinator = new GamePieceCoordinator(intake, hopper, indexers, shooter);
    hubTargetingService = new HubTargetingService(drive, shooter);
    fieldTargetingService = new FieldTargetingService(drive);
    autoRoutines =
        new AutoRoutines(
            drive,
            shooter,
            hopper,
            indexers,
            intake,
            gamePieceCoordinator,
            hubTargetingService,
            fieldTargetingService);
    autoRoutines.registerNamedCommands();
    autoChooser = autoRoutines.buildAutoChooser();

    OperatorActionPublisher operatorActionPublisher = new OperatorActionPublisher();
    autoAssistController = new AutoAssistController(operatorActionPublisher);
    operatorDashboard = new OperatorDashboard(operatorActionPublisher, autoChooser, autoRoutines);
    operatorFeedbackController = new OperatorFeedbackController(driverController);
    new OperatorBindings(
            driverController,
            operatorController,
            drive,
            intake,
            shooter,
            gamePieceCoordinator,
            hubTargetingService,
            fieldTargetingService,
            autoAssistController)
        .configure();

    robotVisualizationPublisher = new RobotVisualizationPublisher(intake, hopper, shooter);
    fieldSimulationManager =
        new FieldSimulationManager(
            drive,
            shooter,
            intake,
            hopper,
            indexers,
            gameState,
            gamePieceCoordinator,
            operatorFeedbackController,
            driveSimulationLocal);

    registerDashboardActions();

    if (Constants.currentMode == Constants.Mode.SIM) {
      fieldSimulationManager.resetField();
    }
  }

  public Command getAutonomousCommand() {
    return autoRoutines.selectAutonomousCommand(autoChooser);
  }

  public void periodic() {
    hubTargetingService.update();
    robotVisualizationPublisher.publish();
    operatorFeedbackController.periodic();
    operatorDashboard.periodic();
  }

  public void onDisabledInit() {
    gamePieceCoordinator.onDisabledInit();
    autoAssistController.onDisabledInit();
    operatorFeedbackController.onDisabledInit();
    fieldSimulationManager.resetField();
  }

  public void updateSimulation() {
    fieldSimulationManager.update();
  }

  private void registerDashboardActions() {
    operatorDashboard.registerTrackedAction(
        "AutoAssist/ParkAtLadderL1",
        autoAssistController
            .scheduleAction(
                "AutoAssist/ParkAtLadderL1", fieldTargetingService::parkAtLadderL1Command)
            .ignoringDisable(true));
    operatorDashboard.registerAction(
        "Shooter/HomeHoodToHardStop",
        "Calibration/Shooter/HomeHoodToHardStop",
        shooter.homeHoodToHardStopCommand());
    operatorDashboard.registerAction(
        "Intake/CalibratePivotToHardStops",
        "Calibration/GamePiece/Intake/CalibratePivotToHardStops",
        intake.calibrateIntakePivotToHardStopsCommand());

    if (Constants.currentMode == Constants.Mode.SIM) {
      operatorDashboard.registerAction(
          "Simulation/ResetField",
          "Simulation/Actions/ResetField",
          Commands.runOnce(fieldSimulationManager::resetField).ignoringDisable(true));
    }
  }
}
