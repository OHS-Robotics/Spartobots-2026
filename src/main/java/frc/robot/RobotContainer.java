// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.body.Agitators;
import frc.robot.subsystems.body.GamePieceManager;
import frc.robot.subsystems.body.Hopper;
import frc.robot.subsystems.body.Intake;
import frc.robot.subsystems.body.shooter.Shooter;
import frc.robot.subsystems.body.shooter.ShooterConstants;
import frc.robot.subsystems.body.shooter.ShooterIO;
import frc.robot.subsystems.body.shooter.ShooterIOSim;
import frc.robot.subsystems.body.shooter.ShooterIOSparkMax;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static final Pose2d SIM_START_POSE = new Pose2d(3.0, 3.0, Rotation2d.kZero);
  private static final double HUB_TOP_LENGTH_Y_METERS = Units.inchesToMeters(47.0);
  private static final double HUB_RAMP_LENGTH_Y_METERS = Units.inchesToMeters(73.0);
  private static final double HUB_WIDTH_X_METERS = Units.inchesToMeters(47.0);
  private static final double HUMP_PEAK_HEIGHT_METERS = Units.inchesToMeters(7.0);
  private static final double HUMP_X_CLEARANCE_MARGIN_METERS = 0.15;
  private static final double FIELD_X_MIN_METERS = 0.0;
  private static final double FIELD_X_MAX_METERS = 16.54105;
  private static final double FIELD_Y_MIN_METERS = 0.0;
  private static final double FIELD_Y_MAX_METERS = 8.06926;
  private static final double WALL_IMPACT_MARGIN_METERS = 0.08;
  private static final double WALL_IMPACT_MIN_APPROACH_SPEED_MPS = 0.55;
  private static final double WALL_IMPACT_ACCEL_THRESHOLD_MPS2 = 9.0;
  private static final double WALL_RUMBLE_DURATION_SECS = 0.22;
  private static final double WALL_RUMBLE_STRENGTH = 1.0;
  private static final double ROBOT_BODY_BASE_HEIGHT_METERS = 0.12;
  private static final double MODULE_HEIGHT_ABOVE_GROUND_METERS = 0.05;
  private static final double MANUAL_HOOD_STEP_DEGREES = 0.35;
  private static final double MANUAL_INTAKE_PIVOT_SPEED = 0.35;
  private static final double MANUAL_HOPPER_EXTENSION_SPEED = 0.40;
  private static final double START_AUTO_OPENING_SHOT_SECONDS = 1.75;
  private static final double START_AUTO_TRENCH_TIMEOUT_SECONDS = 4.0;
  private static final double START_AUTO_OUTPOST_TIMEOUT_SECONDS = 3.0;
  private static final int TOP_LEFT_PADDLE_BUTTON = 7;
  private static final int TOP_RIGHT_PADDLE_BUTTON = 8;
  private static final int BOTTOM_LEFT_PADDLE_BUTTON = 11;
  // Estimated field poses; tune after on-field validation.
  private static final Pose2d BLUE_LADDER_ALIGN_POSE =
      new Pose2d(1.25, Constants.fieldWidth - 0.75, Rotation2d.kZero);
  private static final Pose2d RED_LADDER_ALIGN_POSE =
      new Pose2d(Constants.fieldLength - 1.25, 0.75, Rotation2d.fromDegrees(180.0));
  private static final Pose2d BLUE_DEPOT_ALIGN_POSE =
      new Pose2d(Constants.blueLine - 0.6, Constants.midLineY, Rotation2d.kZero);
  private static final Pose2d RED_DEPOT_ALIGN_POSE =
      new Pose2d(Constants.redLine + 0.6, Constants.midLineY, Rotation2d.fromDegrees(180.0));
  private static final HumpPoseSample FLAT_GROUND_SAMPLE =
      new HumpPoseSample(new double[] {0.0, 0.0, 0.0, 0.0}, 0.0, 0.0, 0.0);

  // Subsystems
  private final Drive drive;
  private final Shooter shooter;
  private final Intake intake;
  private final Hopper hopper;
  private final Agitators agitators;
  private final GamePieceManager gamePieceManager;

  // Controller
  public final CommandXboxController controller = new CommandXboxController(0);
  private Command activeAutoAssistCommand = null;
  private boolean shooterDemandFromAlign = false;
  private boolean shooterDemandFromTrigger = false;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private final Vision vision;
  private SwerveDriveSimulation driveSimulation = null;
  private ChassisSpeeds previousSimFieldSpeeds = null;
  private double rumbleUntilTimestampSeconds = 0.0;
  private int simulatedShooterShotsLaunched = 0;
  private int simulatedShooterHubHits = 0;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    intake = new Intake();
    hopper = new Hopper();
    agitators = new Agitators();

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        shooter = new Shooter(new ShooterIOSparkMax());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0),
                new VisionIOPhotonVision(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1));
        break;

      case SIM:
        // Sim robot, instantiate MapleSim physics implementations
        SimulatedArena.overrideInstance(new Arena2026Rebuilt(false));
        driveSimulation = new SwerveDriveSimulation(DriveConstants.mapleSimConfig, SIM_START_POSE);
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOSim(driveSimulation.getModules()[0]),
                new ModuleIOSim(driveSimulation.getModules()[1]),
                new ModuleIOSim(driveSimulation.getModules()[2]),
                new ModuleIOSim(driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);
        shooter = new Shooter(new ShooterIOSim());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name,
                    VisionConstants.robotToCamera0,
                    driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera1Name,
                    VisionConstants.robotToCamera1,
                    driveSimulation::getSimulatedDriveTrainPose));
        resetSimulationField();
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        shooter = new Shooter(new ShooterIO() {});
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0),
                new VisionIOPhotonVision(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1));
        break;
    }

    gamePieceManager = new GamePieceManager(intake, hopper, agitators);
    gamePieceManager.setShooterReadySupplier(shooter::isReadyToFire);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser.addOption("Start Match (Hub + Trench + Outpost)", startOfMatchAutoRoutine());
    autoChooser.addOption("Hub Opening Shot (Interlocked Feed)", openingHubShotAutoRoutine());
    autoChooser.addOption("Trench Collect (Timed)", trenchCollectAutoRoutine());
    autoChooser.addOption("Outpost Collect (Timed)", outpostCollectAutoRoutine());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Register commands to be used in PathPlanner autos.
    NamedCommands.registerCommand("trench", autoDriveUnderTrenchCommand());
    NamedCommands.registerCommand("outpost", driveToOutpostCommand());
    NamedCommands.registerCommand("collectStart", gamePieceManager.setModeCommand(GamePieceManager.Mode.COLLECT));
    NamedCommands.registerCommand("collectStop", gamePieceManager.setModeCommand(GamePieceManager.Mode.HOLD));
    NamedCommands.registerCommand("feedStart", gamePieceManager.setModeCommand(GamePieceManager.Mode.FEED));
    NamedCommands.registerCommand("feedStop", gamePieceManager.setModeCommand(GamePieceManager.Mode.HOLD));
    NamedCommands.registerCommand(
        "shooterOn",
        Commands.runOnce(() -> shooter.setShotControlEnabled(true))
            .beforeStarting(() -> shooter.setManualHoodOverrideEnabled(false)));
    NamedCommands.registerCommand(
        "shooterOff",
        Commands.runOnce(
            () -> {
              shooter.setShotControlEnabled(false);
              gamePieceManager.requestMode(GamePieceManager.Mode.HOLD);
            }));
    NamedCommands.registerCommand("alignHub", drive.alignToHub(() -> 0.0, () -> 0.0, shooter::getHubAirtimeSeconds));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Left stick = translation, right stick = rotation.
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> controller.getLeftY(),
            () -> controller.getLeftX(),
            () -> -controller.getRightX(),
            () -> false));

    // Left stick press = continue auto sequence.
    controller.leftStick().onTrue(scheduleAutoAssist(this::continueAutoSequenceCommand));
    // Right stick press (hold) = align to hub.
    controller.rightStick().whileTrue(alignToHub());

    // Right trigger = run shooter + interlocked feed while held.
    controller.rightTrigger().whileTrue(runShooterWhileHeldCommand());

    // Driver feed controls.
    controller.x().whileTrue(gamePieceManager.collectWhileHeldCommand());
    controller.a().whileTrue(gamePieceManager.reverseWhileHeldCommand());
    controller.b().onTrue(gamePieceManager.setModeCommand(GamePieceManager.Mode.IDLE));

    // Left bumper = cancel any auto-assist command.
    controller.leftBumper().onTrue(Commands.runOnce(this::cancelAutoAssist));
    // Right bumper = drive under nearest trench.
    controller.rightBumper().onTrue(scheduleAutoAssist(this::autoDriveUnderTrenchCommand));

    // D-pad up/down = manual hood extension with hard stops.
    controller
        .povUp()
        .whileTrue(Commands.run(() -> shooter.adjustHoodSetpointDegrees(MANUAL_HOOD_STEP_DEGREES)));
    controller
        .povDown()
        .whileTrue(
            Commands.run(() -> shooter.adjustHoodSetpointDegrees(-MANUAL_HOOD_STEP_DEGREES)));
    // D-pad left/right = manual intake pivot control.
    controller.povLeft().whileTrue(runIntakePivotWhileHeldCommand(-MANUAL_INTAKE_PIVOT_SPEED));
    controller.povRight().whileTrue(runIntakePivotWhileHeldCommand(MANUAL_INTAKE_PIVOT_SPEED));
    // Y/start = hopper extension out/in.
    controller.y().whileTrue(runHopperExtensionWhileHeldCommand(MANUAL_HOPPER_EXTENSION_SPEED));
    controller.start().whileTrue(runHopperExtensionWhileHeldCommand(-MANUAL_HOPPER_EXTENSION_SPEED));

    // Paddle remaps (configure paddles in Xbox accessories app to emit these buttons).
    new Trigger(() -> controller.getHID().getRawButton(TOP_LEFT_PADDLE_BUTTON))
        .onTrue(scheduleAutoAssist(this::alignToLadderCommand));
    new Trigger(() -> controller.getHID().getRawButton(TOP_RIGHT_PADDLE_BUTTON))
        .onTrue(scheduleAutoAssist(this::driveToOutpostCommand));
    new Trigger(() -> controller.getHID().getRawButton(BOTTOM_LEFT_PADDLE_BUTTON))
        .onTrue(scheduleAutoAssist(this::alignToDepotCommand));

    if (Constants.currentMode == Constants.Mode.SIM) {
      // controller.leftBumper().onTrue(Commands.runOnce(this::resetSimulationField).ignoringDisable(true));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command selectedAuto = autoChooser.get();
    return selectedAuto != null ? selectedAuto : startOfMatchAutoRoutine();
  }

  public Command autoDriveUnderTrenchCommand() {
    return drive.autoDriveUnderTrenchCommand();
  }

  public Command driveToOutpostCommand() {
    return drive.outpostLoadAuto();
  }

  public Command alignToHub() {
    Command driveAlignCommand =
        drive.alignToHub(
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () ->
                shooter
                    .updateHubShotSolution(
                        drive.getPose(),
                        drive.getNearestHubPose(),
                        drive.getFieldRelativeVelocityMetersPerSecond())
                    .airtimeSeconds());
    Command shooterAutoAdjustCommand =
        Commands.startEnd(
            () -> setShooterDemandFromAlign(true), () -> setShooterDemandFromAlign(false));
    return driveAlignCommand.alongWith(shooterAutoAdjustCommand);
  }

  private Command startOfMatchAutoRoutine() {
    return Commands.sequence(
            openingHubShotAutoRoutine(), trenchCollectAutoRoutine(), outpostCollectAutoRoutine())
        .finallyDo(
            () -> {
              shooter.setShotControlEnabled(false);
              gamePieceManager.requestMode(GamePieceManager.Mode.IDLE);
            });
  }

  private Command openingHubShotAutoRoutine() {
    Command openingHubShot =
        Commands.deadline(
            Commands.waitSeconds(START_AUTO_OPENING_SHOT_SECONDS),
            drive.alignToHub(() -> 0.0, () -> 0.0, shooter::getHubAirtimeSeconds),
            Commands.run(
                    () -> {
                      shooter.updateHubShotSolution(
                          drive.getPose(),
                          drive.getNearestHubPose(),
                          drive.getFieldRelativeVelocityMetersPerSecond());
                      shooter.setShotControlEnabled(true);
                      gamePieceManager.requestMode(GamePieceManager.Mode.FEED);
                    },
                    shooter,
                    gamePieceManager)
                .beforeStarting(() -> shooter.setManualHoodOverrideEnabled(false))
                .finallyDo(
                    () -> {
                      shooter.setShotControlEnabled(false);
                      gamePieceManager.requestMode(GamePieceManager.Mode.HOLD);
                    }));
    return openingHubShot;
  }

  private Command trenchCollectAutoRoutine() {
    return Commands.deadline(
        drive.autoDriveUnderTrenchCommand().withTimeout(START_AUTO_TRENCH_TIMEOUT_SECONDS),
        gamePieceManager.collectWhileHeldCommand());
  }

  private Command outpostCollectAutoRoutine() {
    return Commands.deadline(
        drive.outpostLoadAuto().withTimeout(START_AUTO_OUTPOST_TIMEOUT_SECONDS),
        gamePieceManager.collectWhileHeldCommand());
  }

  public Command alignToOutpost() {
    return drive.alignToOutpost(() -> -controller.getLeftX(), () -> -controller.getLeftY());
  }

  public Command alignToLadderCommand() {
    return drive.alignToPose(getAlliancePose(BLUE_LADDER_ALIGN_POSE, RED_LADDER_ALIGN_POSE));
  }

  public Command alignToDepotCommand() {
    return drive.alignToPose(getAlliancePose(BLUE_DEPOT_ALIGN_POSE, RED_DEPOT_ALIGN_POSE));
  }

  private Pose2d getAlliancePose(Pose2d bluePose, Pose2d redPose) {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? redPose : bluePose;
  }

  private Command runIntakePivotWhileHeldCommand(double speed) {
    return Commands.startEnd(
        () -> intake.setIntakePivotSpeed(speed), intake::stopIntakePivot, intake);
  }

  private Command runHopperExtensionWhileHeldCommand(double speed) {
    return Commands.startEnd(
        () -> hopper.setHopperExtensionSpeed(speed), hopper::stopHopperExtension, hopper);
  }

  private Command runShooterWhileHeldCommand() {
    return Commands.parallel(
        Commands.startEnd(
            () -> setShooterDemandFromTrigger(true), () -> setShooterDemandFromTrigger(false)),
        gamePieceManager.feedWhileHeldCommand());
  }

  private void setShooterDemandFromAlign(boolean enabled) {
    shooterDemandFromAlign = enabled;
    refreshShooterControlDemand();
  }

  private void setShooterDemandFromTrigger(boolean enabled) {
    shooterDemandFromTrigger = enabled;
    refreshShooterControlDemand();
  }

  private void refreshShooterControlDemand() {
    shooter.setShotControlEnabled(shooterDemandFromAlign || shooterDemandFromTrigger);
  }

  private Command continueAutoSequenceCommand() {
    return getAutonomousCommand();
  }

  private Command scheduleAutoAssist(Supplier<Command> commandSupplier) {
    return Commands.runOnce(() -> scheduleAutoAssistCommand(commandSupplier.get()));
  }

  private void scheduleAutoAssistCommand(Command command) {
    cancelAutoAssist();
    if (command == null) {
      return;
    }

    final Command[] trackedCommand = new Command[1];
    trackedCommand[0] =
        command.finallyDo(
            () -> {
              if (activeAutoAssistCommand == trackedCommand[0]) {
                activeAutoAssistCommand = null;
              }
            });
    activeAutoAssistCommand = trackedCommand[0];
    activeAutoAssistCommand.schedule();
  }

  private void cancelAutoAssist() {
    if (activeAutoAssistCommand != null) {
      activeAutoAssistCommand.cancel();
      activeAutoAssistCommand = null;
    }
  }

  public void updateHubShotSolution() {
    shooter.updateHubShotSolution(
        drive.getPose(), drive.getNearestHubPose(), drive.getFieldRelativeVelocityMetersPerSecond());
  }

  public void onDisabledInit() {
    shooterDemandFromAlign = false;
    shooterDemandFromTrigger = false;
    refreshShooterControlDemand();
    gamePieceManager.requestMode(GamePieceManager.Mode.IDLE);
    cancelAutoAssist();
    resetSimulationField();
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM || driveSimulation == null) {
      return;
    }

    SimulatedArena arena = SimulatedArena.getInstance();
    drive.setPose(SIM_START_POSE);
    arena.resetFieldForAuto();
    previousSimFieldSpeeds = null;
    rumbleUntilTimestampSeconds = 0.0;
    simulatedShooterShotsLaunched = 0;
    simulatedShooterHubHits = 0;
    controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
    Logger.recordOutput("Shooter/Simulation/ShotsLaunched", simulatedShooterShotsLaunched);
    Logger.recordOutput("Shooter/Simulation/HubHits", simulatedShooterHubHits);
    Logger.recordOutput("Shooter/Simulation/ShotTrajectory", new Pose3d[] {});
    Logger.recordOutput("Shooter/Simulation/ActiveFuelProjectiles", new Pose3d[] {});
    Logger.recordOutput("Shooter/Simulation/LastLaunchSpeedMetersPerSec", 0.0);
    Logger.recordOutput("Shooter/Simulation/LastLaunchAngleDegrees", 0.0);
    Logger.recordOutput("Shooter/Simulation/LastLaunchYawDegrees", 0.0);
    Logger.recordOutput("Shooter/Simulation/LastLaunchPose3d", new Pose3d());
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM || driveSimulation == null) {
      return;
    }

    Pose2d launchRobotPose = driveSimulation.getSimulatedDriveTrainPose();
    ChassisSpeeds launchFieldSpeeds =
        driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative();
    maybeLaunchSimulatedFuel(launchRobotPose, launchFieldSpeeds);

    SimulatedArena arena = SimulatedArena.getInstance();
    arena.simulationPeriodic();
    Pose2d robotPose = driveSimulation.getSimulatedDriveTrainPose();
    ChassisSpeeds simFieldSpeeds =
        driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative();
    updateCollisionRumble(robotPose, simFieldSpeeds);
    HumpPoseSample humpPoseSample = sampleHumpPose(robotPose);
    Logger.recordOutput("FieldSimulation/RobotPose", robotPose);
    Logger.recordOutput(
        "FieldSimulation/RobotPose3d", getSimulatedRobotPose3d(robotPose, humpPoseSample));
    Logger.recordOutput(
        "FieldSimulation/RobotParts/SwerveModules",
        getSimulatedModulePoses(robotPose, humpPoseSample));
    Logger.recordOutput("FieldSimulation/GamePieces/Fuel", arena.getGamePiecesArrayByType("Fuel"));
    Logger.recordOutput(
        "FieldSimulation/GamePieces/Note",
        arena.getGamePiecesArrayByType("Note"));
    Logger.recordOutput(
        "FieldSimulation/GamePieces/Coral",
        arena.getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/GamePieces/Algae",
        arena.getGamePiecesArrayByType("Algae"));
    Logger.recordOutput(
        "Shooter/Simulation/ActiveFuelProjectiles", getActiveFuelProjectilePoses(arena));
    Logger.recordOutput("Shooter/Simulation/ShotsLaunched", simulatedShooterShotsLaunched);
    Logger.recordOutput("Shooter/Simulation/HubHits", simulatedShooterHubHits);
  }

  private void maybeLaunchSimulatedFuel(Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {
    if (!shooter.shouldTriggerSimulatedShot(Timer.getFPGATimestamp())) {
      return;
    }

    Rotation2d shooterFacing = robotPose.getRotation().plus(ShooterConstants.shooterFacingOffset);
    Rotation2d launchPitch = shooter.getMeasuredHoodAngle();
    double launchSpeedMetersPerSec =
        MathUtil.clamp(
            shooter.getEstimatedLaunchSpeedFromMeasuredWheelsMetersPerSec(),
            ShooterConstants.minLaunchSpeedMetersPerSec,
            ShooterConstants.maxLaunchSpeedMetersPerSec);
    Pose2d targetHubPose = drive.getNearestHubPose();

    RebuiltFuelOnFly projectile =
        new RebuiltFuelOnFly(
            robotPose.getTranslation(),
            ShooterConstants.shooterMuzzleOffsetOnRobot,
            fieldRelativeSpeeds,
            shooterFacing,
            Meters.of(shooter.getLaunchHeightMeters()),
            MetersPerSecond.of(launchSpeedMetersPerSec),
            Radians.of(launchPitch.getRadians()));

    projectile
        .withTargetPosition(
            () ->
                new Translation3d(
                    targetHubPose.getX(),
                    targetHubPose.getY(),
                    ShooterConstants.hubCenterHeightMeters))
        .withTargetTolerance(
            new Translation3d(
                ShooterConstants.projectileTargetToleranceXYMeters,
                ShooterConstants.projectileTargetToleranceXYMeters,
                ShooterConstants.projectileTargetToleranceZMeters))
        .withHitTargetCallBack(
            () -> {
              simulatedShooterHubHits++;
              Logger.recordOutput("Shooter/Simulation/HubHits", simulatedShooterHubHits);
            })
        .withProjectileTrajectoryDisplayCallBack(
            trajectory ->
                Logger.recordOutput(
                    "Shooter/Simulation/ShotTrajectory", trajectory.toArray(Pose3d[]::new)));

    SimulatedArena.getInstance().addGamePieceProjectile(projectile);
    simulatedShooterShotsLaunched++;

    Logger.recordOutput("Shooter/Simulation/ShotsLaunched", simulatedShooterShotsLaunched);
    Logger.recordOutput("Shooter/Simulation/LastLaunchSpeedMetersPerSec", launchSpeedMetersPerSec);
    Logger.recordOutput("Shooter/Simulation/LastLaunchAngleDegrees", launchPitch.getDegrees());
    Logger.recordOutput("Shooter/Simulation/LastLaunchYawDegrees", shooterFacing.getDegrees());
    Logger.recordOutput(
        "Shooter/Simulation/LastLaunchPose3d",
        new Pose3d(
            robotPose.getX(),
            robotPose.getY(),
            shooter.getLaunchHeightMeters(),
            new Rotation3d(0.0, -launchPitch.getRadians(), shooterFacing.getRadians())));
  }

  private void updateCollisionRumble(Pose2d robotPose, ChassisSpeeds currentFieldSpeeds) {
    double nowSeconds = Timer.getFPGATimestamp();

    if (previousSimFieldSpeeds != null) {
      double deltaVx =
          currentFieldSpeeds.vxMetersPerSecond - previousSimFieldSpeeds.vxMetersPerSecond;
      double deltaVy =
          currentFieldSpeeds.vyMetersPerSecond - previousSimFieldSpeeds.vyMetersPerSecond;
      double translationalAccelerationMetersPerSecSquared = Math.hypot(deltaVx, deltaVy) / 0.02;

      boolean approachingWall = isApproachingFieldWall(robotPose, previousSimFieldSpeeds);
      boolean impactDetected =
          approachingWall
              && translationalAccelerationMetersPerSecSquared >= WALL_IMPACT_ACCEL_THRESHOLD_MPS2;

      if (impactDetected) {
        rumbleUntilTimestampSeconds = nowSeconds + WALL_RUMBLE_DURATION_SECS;
      }

      Logger.recordOutput("FieldSimulation/WallImpact/ApproachingWall", approachingWall);
      Logger.recordOutput(
          "FieldSimulation/WallImpact/TranslationalAccelerationMps2",
          translationalAccelerationMetersPerSecSquared);
      Logger.recordOutput("FieldSimulation/WallImpact/Detected", impactDetected);
    }

    previousSimFieldSpeeds = currentFieldSpeeds;
    double rumbleStrength = nowSeconds < rumbleUntilTimestampSeconds ? WALL_RUMBLE_STRENGTH : 0.0;
    controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, rumbleStrength);
    Logger.recordOutput("FieldSimulation/WallImpact/RumbleStrength", rumbleStrength);
  }

  private Pose3d[] getActiveFuelProjectilePoses(SimulatedArena arena) {
    return arena.gamePieceLaunched().stream()
        .filter(projectile -> "Fuel".equals(projectile.getType()))
        .map(projectile -> projectile.getPose3d())
        .toArray(Pose3d[]::new);
  }

  private boolean isApproachingFieldWall(Pose2d robotPose, ChassisSpeeds fieldSpeeds) {
    double halfLength = DriveConstants.bumperLengthXMeters * 0.5;
    double halfWidth = DriveConstants.bumperWidthYMeters * 0.5;

    boolean nearMinX =
        robotPose.getX() <= FIELD_X_MIN_METERS + halfLength + WALL_IMPACT_MARGIN_METERS;
    boolean nearMaxX =
        robotPose.getX() >= FIELD_X_MAX_METERS - halfLength - WALL_IMPACT_MARGIN_METERS;
    boolean nearMinY =
        robotPose.getY() <= FIELD_Y_MIN_METERS + halfWidth + WALL_IMPACT_MARGIN_METERS;
    boolean nearMaxY =
        robotPose.getY() >= FIELD_Y_MAX_METERS - halfWidth - WALL_IMPACT_MARGIN_METERS;

    boolean approachingMinX =
        nearMinX && fieldSpeeds.vxMetersPerSecond < -WALL_IMPACT_MIN_APPROACH_SPEED_MPS;
    boolean approachingMaxX =
        nearMaxX && fieldSpeeds.vxMetersPerSecond > WALL_IMPACT_MIN_APPROACH_SPEED_MPS;
    boolean approachingMinY =
        nearMinY && fieldSpeeds.vyMetersPerSecond < -WALL_IMPACT_MIN_APPROACH_SPEED_MPS;
    boolean approachingMaxY =
        nearMaxY && fieldSpeeds.vyMetersPerSecond > WALL_IMPACT_MIN_APPROACH_SPEED_MPS;

    return approachingMinX || approachingMaxX || approachingMinY || approachingMaxY;
  }

  private Pose3d getSimulatedRobotPose3d(Pose2d robotPose, HumpPoseSample humpPoseSample) {
    return new Pose3d(
        new Translation3d(
            robotPose.getX(),
            robotPose.getY(),
            ROBOT_BODY_BASE_HEIGHT_METERS + humpPoseSample.heightMeters()),
        new Rotation3d(
            humpPoseSample.rollRadians(),
            humpPoseSample.pitchRadians(),
            robotPose.getRotation().getRadians()));
  }

  private Pose3d[] getSimulatedModulePoses(Pose2d robotPose, HumpPoseSample humpPoseSample) {
    Pose3d[] modulePoses = new Pose3d[DriveConstants.moduleTranslations.length];
    for (int i = 0; i < modulePoses.length; i++) {
      Translation2d moduleOffset =
          DriveConstants.moduleTranslations[i].rotateBy(robotPose.getRotation());
      Translation2d modulePosition = robotPose.getTranslation().plus(moduleOffset);
      Rotation2d moduleRotation =
          driveSimulation.getModules()[i].getSteerAbsoluteFacing().plus(robotPose.getRotation());

      modulePoses[i] =
          new Pose3d(
              new Translation3d(
                  modulePosition.getX(),
                  modulePosition.getY(),
                  MODULE_HEIGHT_ABOVE_GROUND_METERS + humpPoseSample.moduleHeightsMeters()[i]),
              new Rotation3d(
                  humpPoseSample.rollRadians(),
                  humpPoseSample.pitchRadians(),
                  moduleRotation.getRadians()));
    }
    return modulePoses;
  }

  private HumpPoseSample sampleHumpPose(Pose2d robotPose) {
    double[] moduleHeights = new double[DriveConstants.moduleTranslations.length];
    for (int i = 0; i < moduleHeights.length; i++) {
      Translation2d moduleFieldPosition =
          robotPose
              .getTranslation()
              .plus(DriveConstants.moduleTranslations[i].rotateBy(robotPose.getRotation()));
      moduleHeights[i] = sampleGroundHeightAt(moduleFieldPosition);
    }

    double frontAverage = (moduleHeights[0] + moduleHeights[1]) * 0.5;
    double backAverage = (moduleHeights[2] + moduleHeights[3]) * 0.5;
    double leftAverage = (moduleHeights[0] + moduleHeights[2]) * 0.5;
    double rightAverage = (moduleHeights[1] + moduleHeights[3]) * 0.5;
    double averageHeight =
        (moduleHeights[0] + moduleHeights[1] + moduleHeights[2] + moduleHeights[3]) * 0.25;

    double rollRadians = Math.atan2(leftAverage - rightAverage, DriveConstants.trackWidth);
    double pitchRadians = Math.atan2(frontAverage - backAverage, DriveConstants.wheelBase);
    return new HumpPoseSample(moduleHeights, averageHeight, rollRadians, pitchRadians);
  }

  private double sampleGroundHeightAt(Translation2d position) {
    double blueHeight = sampleSingleHumpHeight(Constants.blueHub.getX(), position);
    double redHeight = sampleSingleHumpHeight(Constants.redHub.getX(), position);
    return Math.max(blueHeight, redHeight);
  }

  private double sampleSingleHumpHeight(double hubCenterXMeters, Translation2d position) {
    double maxHumpXDistance = (HUB_WIDTH_X_METERS / 2.0) + HUMP_X_CLEARANCE_MARGIN_METERS;
    if (Math.abs(position.getX() - hubCenterXMeters) > maxHumpXDistance) {
      return 0.0;
    }

    double yOffsetFromHub = position.getY() - Constants.blueHub.getY();
    double absYOffset = Math.abs(yOffsetFromHub);
    double halfTopLength = HUB_TOP_LENGTH_Y_METERS / 2.0;
    if (absYOffset > (halfTopLength + HUB_RAMP_LENGTH_Y_METERS)) {
      return 0.0;
    }

    if (absYOffset <= halfTopLength) {
      return HUMP_PEAK_HEIGHT_METERS;
    }

    double rampTravelMeters = absYOffset - halfTopLength;
    double rampPercent =
        1.0 - MathUtil.clamp(rampTravelMeters / HUB_RAMP_LENGTH_Y_METERS, 0.0, 1.0);
    return HUMP_PEAK_HEIGHT_METERS * rampPercent;
  }

  private record HumpPoseSample(
      double[] moduleHeightsMeters, double heightMeters, double rollRadians, double pitchRadians) {}
}
