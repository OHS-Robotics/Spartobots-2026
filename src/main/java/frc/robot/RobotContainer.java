// Copyright (c) 2021-2026 Littleton Robotics
// Copyright (c) 2026 Team 4687 Spartobots
//
// SPDX-License-Identifier: BSD-3-Clause

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.auto.AutoRoutines;
import frc.robot.commands.DriveCommands;
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
import frc.robot.subsystems.vision.VisionIO;
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
  // Temporary bringup switches to keep missing hardware from flooding the CAN bus.
  private static final boolean enableRealGamePieceHardware = true;
  private static final boolean enableRealVisionHardware = false;
  private static final int visionPoseSyncMinTagCount = 2;
  private static final double visionPoseSyncMaxAgeSeconds = 0.25;
  private static final double visionPoseSyncMaxAverageTagDistanceMeters = 4.5;
  private static final double visionPoseSyncMaxLinearSpeedMetersPerSec = 0.20;
  private static final double visionPoseSyncMaxAngularSpeedRadPerSec = Units.degreesToRadians(20.0);
  private static final double visionPoseSyncMinTranslationErrorMeters = 0.08;
  private static final double visionPoseSyncMinRotationErrorRadians = Units.degreesToRadians(3.0);
  private static final double visionPoseSyncCooldownSeconds = 0.5;

  private final Drive drive;
  private final Shooter shooter;
  private final Intake intake;
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
  private double lastVisionPoseSyncTimestampSeconds = Double.NEGATIVE_INFINITY;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    Drive driveLocal;
    Shooter shooterLocal;
    Intake intakeLocal;
    Indexers indexersLocal;
    Vision visionLocal;
    SwerveDriveSimulation driveSimulationLocal = null;

    switch (Constants.currentMode) {
      case REAL:
        intakeLocal =
            enableRealGamePieceHardware ? new Intake(new IntakeIOSparkMax()) : new Intake();
        indexersLocal =
            enableRealGamePieceHardware ? new Indexers(new IndexersIOSparkMax()) : new Indexers();
        driveLocal =
            new Drive(
                new GyroIONavX(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        shooterLocal =
            enableRealGamePieceHardware ? new Shooter(new ShooterIOSparkMax()) : new Shooter();
        visionLocal =
            new Vision(
                driveLocal::addVisionMeasurement,
                driveLocal::getPose,
                enableRealVisionHardware
                    ? new VisionIOPhotonVision(
                        VisionConstants.camera0Name,
                        VisionConstants.robotToCamera0,
                        VisionConstants.camera0Pipeline)
                    : new VisionIO() {},
                enableRealVisionHardware
                    ? new VisionIOPhotonVision(
                        VisionConstants.camera1Name,
                        VisionConstants.robotToCamera1,
                        VisionConstants.camera1Pipeline)
                    : new VisionIO() {});
        break;

      case SIM:
        SimulatedArena.overrideInstance(new Arena2026Rebuilt(false));
        driveSimulationLocal =
            new SwerveDriveSimulation(
                DriveConstants.getMapleSimConfig(), Constants.simulationStartPose);
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulationLocal);
        intakeLocal = new Intake(new IntakeIOSim());
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
    indexers = indexersLocal;
    gameState = new GameStateSubsystem();
    vision = visionLocal;

    gamePieceCoordinator = new GamePieceCoordinator(intake, indexers, shooter);
    hubTargetingService = new HubTargetingService(drive, shooter);
    fieldTargetingService = new FieldTargetingService(drive);
    autoRoutines =
        new AutoRoutines(
            drive,
            shooter,
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

    robotVisualizationPublisher = new RobotVisualizationPublisher(intake, shooter);
    fieldSimulationManager =
        new FieldSimulationManager(
            drive,
            shooter,
            intake,
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
    syncDrivePoseToVisionIfNeeded();
    shooter.setTrenchSafetyRetractOverrideEnabled(fieldTargetingService.isRobotNearTrench());
    hubTargetingService.update();
    robotVisualizationPublisher.publish();
    operatorFeedbackController.periodic();
    operatorDashboard.periodic();
  }

  public void onDisabledInit() {
    disableMechanismCalibrationModes();
    gamePieceCoordinator.onDisabledInit();
    autoAssistController.onDisabledInit();
    operatorFeedbackController.onDisabledInit();
    fieldSimulationManager.resetField();
  }

  public void onTeleopInit() {
    disableMechanismCalibrationModes();
  }

  public void updateSimulation() {
    fieldSimulationManager.update();
  }

  private void syncDrivePoseToVisionIfNeeded() {
    var estimate = vision.getBestRobotPoseEstimate();
    if (estimate.isEmpty()) {
      return;
    }

    double nowSeconds = Timer.getFPGATimestamp();
    var poseEstimate = estimate.get();
    if (nowSeconds - poseEstimate.timestampSeconds() > visionPoseSyncMaxAgeSeconds) {
      return;
    }
    if (poseEstimate.tagCount() < visionPoseSyncMinTagCount) {
      return;
    }
    if (poseEstimate.averageTagDistanceMeters() > visionPoseSyncMaxAverageTagDistanceMeters) {
      return;
    }
    if (nowSeconds - lastVisionPoseSyncTimestampSeconds < visionPoseSyncCooldownSeconds) {
      return;
    }

    boolean disabled = DriverStation.isDisabled();
    double linearSpeedMetersPerSec = drive.getFieldRelativeVelocityMetersPerSecond().getNorm();
    double angularSpeedRadPerSec = Math.abs(drive.getYawVelocityRadPerSec());
    if (!disabled
        && (linearSpeedMetersPerSec > visionPoseSyncMaxLinearSpeedMetersPerSec
            || angularSpeedRadPerSec > visionPoseSyncMaxAngularSpeedRadPerSec)) {
      return;
    }

    double translationErrorMeters =
        drive.getPose().getTranslation().getDistance(poseEstimate.pose().getTranslation());
    double rotationErrorRadians =
        Math.abs(
            MathUtil.angleModulus(
                drive.getRotation().minus(poseEstimate.pose().getRotation()).getRadians()));
    if (translationErrorMeters < visionPoseSyncMinTranslationErrorMeters
        && rotationErrorRadians < visionPoseSyncMinRotationErrorRadians) {
      return;
    }

    drive.setPose(poseEstimate.pose());
    lastVisionPoseSyncTimestampSeconds = nowSeconds;
  }

  private void disableMechanismCalibrationModes() {
    shooter.setCalibrationModeEnabled(false);
    intake.setCalibrationModeEnabled(false);
    indexers.setCalibrationModeEnabled(false);
  }

  private void registerDashboardActions() {
    operatorDashboard.registerAction(
        "Shooter/HomeHoodToHardStop",
        "Calibration/Shooter/HomeHoodToHardStop",
        shooter.homeHoodToHardStopCommand());
    operatorDashboard.registerAction(
        "Shooter/Calibration/EnableMode",
        "Calibration/Shooter/EnableCalibrationMode",
        shooter.enableCalibrationModeCommand());
    operatorDashboard.registerAction(
        "Shooter/Calibration/DisableMode",
        "Calibration/Shooter/DisableCalibrationMode",
        shooter.disableCalibrationModeCommand());
    operatorDashboard.registerAction(
        "Shooter/Calibration/RecordSample",
        "Calibration/Shooter/RecordCalibrationSample",
        shooter.recordCalibrationSampleCommand());
    operatorDashboard.registerAction(
        "Intake/CalibratePivotToHardStops",
        "Calibration/GamePiece/Intake/CalibratePivotToHardStops",
        intake.calibrateIntakePivotToHardStopsCommand());
    operatorDashboard.registerAction(
        "Intake/Calibration/EnableMode",
        "Calibration/GamePiece/Intake/EnableClosedLoopMode",
        intake.enableCalibrationModeCommand());
    operatorDashboard.registerAction(
        "Intake/Calibration/DisableMode",
        "Calibration/GamePiece/Intake/DisableClosedLoopMode",
        intake.disableCalibrationModeCommand());
    operatorDashboard.registerAction(
        "Indexers/Calibration/EnableMode",
        "Calibration/GamePiece/Indexers/EnableClosedLoopMode",
        indexers.enableCalibrationModeCommand());
    operatorDashboard.registerAction(
        "Indexers/Calibration/DisableMode",
        "Calibration/GamePiece/Indexers/DisableClosedLoopMode",
        indexers.disableCalibrationModeCommand());
    operatorDashboard.registerAction(
        "Drive/WheelRadiusCharacterization",
        "Tuning/Drive/WheelRadiusCharacterization",
        DriveCommands.wheelRadiusCharacterization(drive));
    operatorDashboard.registerAction(
        "Drive/SimpleFFCharacterization",
        "Tuning/Drive/SimpleFFCharacterization",
        DriveCommands.feedforwardCharacterization(drive));
    operatorDashboard.registerAction(
        "Drive/SysId/QuasistaticForward",
        "Tuning/Drive/SysId/QuasistaticForward",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    operatorDashboard.registerAction(
        "Drive/SysId/QuasistaticReverse",
        "Tuning/Drive/SysId/QuasistaticReverse",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    operatorDashboard.registerAction(
        "Drive/SysId/DynamicForward",
        "Tuning/Drive/SysId/DynamicForward",
        drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    operatorDashboard.registerAction(
        "Drive/SysId/DynamicReverse",
        "Tuning/Drive/SysId/DynamicReverse",
        drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    operatorDashboard.registerAction(
        "AutoAssist/Paths/BumpLeftIn",
        "AutoAssist/Paths/BumpLeftIn",
        drive.followNamedPath("Bump Left In"));
    operatorDashboard.registerAction(
        "AutoAssist/Paths/BumpLeftOut",
        "AutoAssist/Paths/BumpLeftOut",
        drive.followNamedPath("Bump Left Out"));
    operatorDashboard.registerAction(
        "AutoAssist/Paths/BumpRightIn",
        "AutoAssist/Paths/BumpRightIn",
        drive.followNamedPath("Bump Right In"));
    operatorDashboard.registerAction(
        "AutoAssist/Paths/BumpRightOut",
        "AutoAssist/Paths/BumpRightOut",
        drive.followNamedPath("Bump Right Out"));
    operatorDashboard.registerAction(
        "AutoAssist/Collect/Depot",
        "AutoAssist/Collect/Depot",
        buildCollectAlongPathCommand("Depot"));
    operatorDashboard.registerAction(
        "AutoAssist/Collect/DepotAlt",
        "AutoAssist/Collect/DepotAlt",
        buildCollectAlongPathCommand("Depot Alt"));
    operatorDashboard.registerAction(
        "AutoAssist/Collect/Middle",
        "AutoAssist/Collect/Middle",
        buildCollectAlongPathCommand("Middle"));

    if (Constants.currentMode == Constants.Mode.SIM) {
      operatorDashboard.registerAction(
          "Simulation/ResetField",
          "Simulation/Actions/ResetField",
          Commands.runOnce(fieldSimulationManager::resetField).ignoringDisable(true));
    }
  }

  private Command buildCollectAlongPathCommand(String pathName) {
    return Commands.deadline(
            drive.followNamedPath(pathName),
            Commands.run(() -> gamePieceCoordinator.applyBasicCollect(true), intake, indexers))
        .finallyDo(gamePieceCoordinator::stopGamePieceFlow);
  }
}
