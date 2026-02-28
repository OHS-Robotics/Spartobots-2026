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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.game.GameStateSubsystem;
import frc.robot.subsystems.body.Hopper;
import frc.robot.subsystems.body.HopperIO;
import frc.robot.subsystems.body.HopperIOSim;
import frc.robot.subsystems.body.HopperIOSparkMax;
import frc.robot.subsystems.body.Indexers;
import frc.robot.subsystems.body.IndexersIO;
import frc.robot.subsystems.body.IndexersIOSim;
import frc.robot.subsystems.body.IndexersIOSparkMax;
import frc.robot.subsystems.body.Intake;
import frc.robot.subsystems.body.IntakeIO;
import frc.robot.subsystems.body.IntakeIOSim;
import frc.robot.subsystems.body.IntakeIOSparkMax;
import frc.robot.subsystems.body.ShooterFeedInterlock;
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
  private static final double ENDGAME_WINDOW_START_SECONDS = 30.0;
  private static final double ENDGAME_RUMBLE_DURATION_SECS = 0.65;
  private static final double ENDGAME_RUMBLE_STRENGTH = 0.65;
  private static final String ROBOT_COMPONENTS_LOG_KEY = "AdvantageScope/Robot/Components";
  private static final int COMPONENT_INDEX_INTAKE_PIVOT = 0;
  private static final int COMPONENT_INDEX_HOPPER_EXTENSION = 1;
  private static final int COMPONENT_INDEX_SHOOTER_HOOD = 2;
  private static final int ROBOT_COMPONENT_COUNT = 3;
  private static final Translation3d INTAKE_PIVOT_ORIGIN_ON_ROBOT =
      new Translation3d(0.305, 0.0, 0.215);
  private static final Rotation2d INTAKE_PIVOT_RETRACTED_ANGLE = Rotation2d.fromDegrees(0.0);
  private static final Rotation2d INTAKE_PIVOT_EXTENDED_ANGLE = Rotation2d.fromDegrees(-82.0);
  private static final Translation3d HOPPER_EXTENSION_RETRACTED_ORIGIN_ON_ROBOT =
      new Translation3d(-0.05, 0.0, 0.24);
  private static final Translation3d HOPPER_EXTENSION_AXIS_ON_ROBOT =
      new Translation3d(0.20, 0.0, 0.0);
  private static final Translation3d SHOOTER_HOOD_PIVOT_ON_ROBOT =
      new Translation3d(-0.23, 0.0, 0.56);
  private static final Rotation2d SHOOTER_HOOD_MODEL_PITCH_OFFSET = Rotation2d.kZero;
  private static final double ROBOT_BODY_BASE_HEIGHT_METERS = 0.12;
  private static final double MODULE_HEIGHT_ABOVE_GROUND_METERS = 0.05;
  private static final double MANUAL_HOOD_STEP_DEGREES = 0.35;
  private static final double SHOOTER_TRIGGER_DEADBAND = 0.02;
  // Temporary bring-up bindings for mechanism calibration.
  // Set false after intake/hopper calibration is complete.
  public static final boolean ENABLE_MECHANISM_BRINGUP_BINDINGS = false;
  private static final double INTAKE_PIVOT_BRINGUP_SPEED = 0.25;
  private static final double HOPPER_EXTENSION_BRINGUP_SPEED = 0.25;
  private static final double TOP_INDEXER_MANUAL_FEED_SPEED = 0.45;
  private static final double BOTTOM_INDEXER_BREAKAWAY_SPEED = 1.0; // CCW
  private static final double BOTTOM_INDEXER_HOLD_SPEED = 0.9; // CCW
  private static final double BOTTOM_INDEXER_BREAKAWAY_SECONDS = 0.40;
  private static final double BASIC_COLLECT_INTAKE_SPEED = 0.65;
  private static final double BASIC_COLLECT_AGITATOR_SPEED = 0.55;
  private static final double BASIC_COLLECT_INDEXER_SPEED = 0.55;
  private static final double BASIC_FEED_AGITATOR_SPEED = 0.75;
  private static final double BASIC_FEED_INDEXER_SPEED = 0.75;
  private static final double BASIC_REVERSE_SPEED = -0.45;
  private static final double START_AUTO_OPENING_SHOT_SECONDS = 1.75;
  private static final double START_AUTO_TRENCH_TIMEOUT_SECONDS = 4.0;
  private static final double START_AUTO_OUTPOST_TIMEOUT_SECONDS = 3.0;
  private static final double START_AUTO_OUTPOST_TO_SHOOT_TIMEOUT_SECONDS = 4.0;
  private static final double START_AUTO_OUTPOST_SHOOT_SECONDS = 4.0;
  private static final double START_AUTO_LADDER_ALIGN_TIMEOUT_SECONDS = 4.0;
  private static final int DRIVER_CONTROLLER_PORT = 0;
  private static final int OPERATOR_CONTROLLER_PORT = 1;
  private static final int TOP_LEFT_PADDLE_BUTTON = 7;
  private static final int TOP_RIGHT_PADDLE_BUTTON = 8;
  private static final int BOTTOM_LEFT_PADDLE_BUTTON = 11;
  // Estimated field poses; tune after on-field validation.
  private static final Pose2d BLUE_OUTPOST_OPENING_SHOT_POSE =
      new Pose2d(2.85, 2.1, Rotation2d.fromDegrees(-130.0));
  private static final Pose2d RED_OUTPOST_OPENING_SHOT_POSE =
      new Pose2d(
          Constants.fieldLength - 2.85, Constants.fieldWidth - 2.1, Rotation2d.fromDegrees(50.0));
  private static final Pose2d BLUE_LADDER_ALIGN_POSE =
      new Pose2d(1.25, Constants.fieldWidth - 0.75, Rotation2d.kZero);
  private static final Pose2d RED_LADDER_ALIGN_POSE =
      new Pose2d(Constants.fieldLength - 1.25, 0.75, Rotation2d.fromDegrees(180.0));
  private static final Pose2d BLUE_DEPOT_ALIGN_POSE =
      new Pose2d(Constants.blueLine - 0.6, Constants.midLineY, Rotation2d.kZero);
  private static final Pose2d RED_DEPOT_ALIGN_POSE =
      new Pose2d(Constants.redLine + 0.6, Constants.midLineY, Rotation2d.fromDegrees(180.0));
  // Subsystems
  private final Drive drive;
  private final Shooter shooter;
  private final Intake intake;
  private final Hopper hopper;
  private final Indexers indexers;
  private final GameStateSubsystem gameState;

  @SuppressWarnings("unused")
  private final Vision vision;

  // Controllers
  public final CommandXboxController driverController =
      new CommandXboxController(DRIVER_CONTROLLER_PORT);
  public final CommandXboxController operatorController =
      new CommandXboxController(OPERATOR_CONTROLLER_PORT);
  private Command activeAutoAssistCommand = null;
  private boolean shooterDemandFromAlign = false;
  private double shooterDemandFromTriggerThrottle = 0.0;
  private double bottomIndexerBreakawayUntilTimestampSeconds = 0.0;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private SwerveDriveSimulation driveSimulation = null;
  private ChassisSpeeds previousSimFieldSpeeds = null;
  private double rumbleUntilTimestampSeconds = 0.0;
  private double endgameRumbleUntilTimestampSeconds = 0.0;
  private boolean endgameWindowLatched = false;
  private int simulatedShooterShotsLaunched = 0;
  private int simulatedShooterHubHits = 0;
  private int simulatedShooterShotsActiveHub = 0;
  private int simulatedShooterShotsInactiveHub = 0;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    Drive driveLocal;
    Shooter shooterLocal;
    Intake intakeLocal;
    Hopper hopperLocal;
    Indexers indexersLocal;
    Vision visionLocal;

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
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
        intakeLocal = new Intake(new IntakeIOSim());
        hopperLocal = new Hopper(new HopperIOSim());
        indexersLocal = new Indexers(new IndexersIOSim());
        driveLocal =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOSim(driveSimulation.getModules()[0]),
                new ModuleIOSim(driveSimulation.getModules()[1]),
                new ModuleIOSim(driveSimulation.getModules()[2]),
                new ModuleIOSim(driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);
        shooterLocal = new Shooter(new ShooterIOSim());
        visionLocal =
            new Vision(
                driveLocal::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name,
                    VisionConstants.robotToCamera0,
                    driveSimulation::getSimulatedDriveTrainPose,
                    VisionConstants.camera0SimConfig),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera1Name,
                    VisionConstants.robotToCamera1,
                    driveSimulation::getSimulatedDriveTrainPose,
                    VisionConstants.camera1SimConfig));
        break;

      default:
        // Replayed robot, disable IO implementations
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
                new VisionIOPhotonVision(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0),
                new VisionIOPhotonVision(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1));
        break;
    }

    drive = driveLocal;
    shooter = shooterLocal;
    intake = intakeLocal;
    hopper = hopperLocal;
    indexers = indexersLocal;
    gameState = new GameStateSubsystem();
    vision = visionLocal;

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser.addOption(
        "Start Match (Outpost -> Shoot -> Ladder)", outpostStartShootAndLadderAutoRoutine());
    autoChooser.addOption("Start Match (Hub + Trench + Outpost)", startOfMatchAutoRoutine());
    autoChooser.addOption("Hub Opening Shot (Basic Feed)", openingHubShotAutoRoutine());
    autoChooser.addOption("Trench Collect (Timed)", trenchCollectAutoRoutine());
    autoChooser.addOption("Trench Collect (Timed) new thing", trenchCollectAutoRoutineNew());
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
    NamedCommands.registerCommand(
        "collectStart", Commands.runOnce(() -> applyBasicCollect(true), intake, hopper, indexers));
    NamedCommands.registerCommand(
        "collectStop", Commands.runOnce(this::stopGamePieceFlow, intake, hopper, indexers));
    NamedCommands.registerCommand(
        "feedStart", Commands.runOnce(() -> applyBasicFeed(true), intake, hopper, indexers));
    NamedCommands.registerCommand(
        "feedStop", Commands.runOnce(this::stopGamePieceFlow, intake, hopper, indexers));
    NamedCommands.registerCommand(
        "shooterOn",
        Commands.runOnce(() -> shooter.setShotControlEnabled(true))
            .beforeStarting(() -> shooter.setManualHoodOverrideEnabled(false)));
    NamedCommands.registerCommand(
        "shooterOff",
        Commands.runOnce(
            () -> {
              shooter.setShotControlEnabled(false);
              stopGamePieceFlow();
            },
            intake,
            hopper,
            indexers));
    NamedCommands.registerCommand(
        "alignHub",
        drive.alignToHub(() -> 0.0, () -> 0.0, this::updateHubShotSolutionAndGetAirtimeSeconds));
    NamedCommands.registerCommand("homeHood", shooter.homeHoodToHardStopCommand());

    // Configure the button bindings
    configureButtonBindings();
    SmartDashboard.putData(
        "AutoAssist/ParkAtLadderL1",
        scheduleAutoAssist(this::parkAtLadderL1Command).ignoringDisable(true));
    SmartDashboard.putData("Shooter/HomeHoodToHardStop", shooter.homeHoodToHardStopCommand());

    if (Constants.currentMode == Constants.Mode.SIM) {
      SmartDashboard.putData(
          "Simulation/ResetField",
          Commands.runOnce(this::resetSimulationField).ignoringDisable(true));
      resetSimulationField();
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    configureDriverBindings();
    configureOperatorBindings();
  }

  private void configureDriverBindings() {
    // Left stick = translation, right stick = rotation.
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> driverController.getLeftY(),
            () -> driverController.getLeftX(),
            () -> -driverController.getRightX(),
            () -> false));

    // Right stick press (hold) = align to hub.
    driverController.rightStick().whileTrue(alignToHub());

    // Right trigger = run shooter while held.
    new Trigger(() -> driverController.getRightTriggerAxis() > SHOOTER_TRIGGER_DEADBAND)
        .whileTrue(runShooterDemandWhileHeldCommand());
    // Left trigger = run manual feed + manual indexers while held.
    driverController.leftTrigger().whileTrue(runManualFeedAndIndexersWhileHeldCommand());

    // Left bumper = cancel any auto-assist command.
    driverController.leftBumper().onTrue(Commands.runOnce(this::cancelAutoAssist));
    // Right bumper = drive under nearest trench.
    driverController.rightBumper().onTrue(scheduleAutoAssist(this::autoDriveUnderTrenchCommand));
    // Left stick press = park assist to ladder (L1 endgame alignment).
    driverController.leftStick().onTrue(scheduleAutoAssist(this::parkAtLadderL1Command));

    if (!ENABLE_MECHANISM_BRINGUP_BINDINGS) {
      // Paddle remaps (configure paddles in Xbox accessories app to emit these buttons).
      new Trigger(() -> driverController.getHID().getRawButton(TOP_LEFT_PADDLE_BUTTON))
          .onTrue(scheduleAutoAssist(this::parkAtLadderL1Command));
      new Trigger(() -> driverController.getHID().getRawButton(TOP_RIGHT_PADDLE_BUTTON))
          .onTrue(scheduleAutoAssist(this::driveToOutpostCommand));
      new Trigger(() -> driverController.getHID().getRawButton(BOTTOM_LEFT_PADDLE_BUTTON))
          .onTrue(scheduleAutoAssist(this::alignToDepotCommand));
    }

    if (Constants.currentMode == Constants.Mode.SIM) {
      // driverController.leftBumper().onTrue(Commands.runOnce(this::resetSimulationField).ignoringDisable(true));
    }
  }

  private void configureOperatorBindings() {
    // Game-piece flow controls: Y = collect with indexers, X = collect without indexers,
    // A = reverse, B = stop all game-piece motors.
    operatorController.y().whileTrue(basicCollectWhileHeldCommand(true));
    operatorController.x().whileTrue(basicCollectWhileHeldCommand(false));
    operatorController.a().whileTrue(basicReverseWhileHeldCommand());
    operatorController
        .b()
        .onTrue(Commands.runOnce(this::stopGamePieceFlow, intake, hopper, indexers));

    // D-pad up/down = manual hood extension with hard stops.
    operatorController
        .povUp()
        .whileTrue(Commands.run(() -> shooter.adjustHoodSetpointDegrees(MANUAL_HOOD_STEP_DEGREES)));
    operatorController
        .povDown()
        .whileTrue(
            Commands.run(() -> shooter.adjustHoodSetpointDegrees(-MANUAL_HOOD_STEP_DEGREES)));

    if (ENABLE_MECHANISM_BRINGUP_BINDINGS) {
      // D-pad left/right = manual intake pivot jog.
      operatorController
          .povLeft()
          .whileTrue(runIntakePivotWhileHeldCommand(-INTAKE_PIVOT_BRINGUP_SPEED));
      operatorController
          .povRight()
          .whileTrue(runIntakePivotWhileHeldCommand(INTAKE_PIVOT_BRINGUP_SPEED));

      // Back/start = manual L1 climber jog (legacy Hopper extension control path).
      // These override paddle remap bindings while bring-up mode is enabled.
      operatorController
          .back()
          .whileTrue(runHopperExtensionWhileHeldCommand(-HOPPER_EXTENSION_BRINGUP_SPEED));
      operatorController
          .start()
          .whileTrue(runHopperExtensionWhileHeldCommand(HOPPER_EXTENSION_BRINGUP_SPEED));
    } else {
      operatorController
          .back()
          .onTrue(Commands.runOnce(this::stopGamePieceFlow, intake, hopper, indexers));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command selectedAuto = autoChooser.get();
    return selectedAuto != null ? selectedAuto : outpostStartShootAndLadderAutoRoutine();
  }

  public Command autoDriveUnderTrenchCommand() {
    return drive.autoDriveUnderTrenchCommand();
  }

  public Command driveToOutpostCommand() {
    return drive.driveToOutpostCommand();
  }

  public Command alignToHub() {
    Command driveAlignCommand =
        drive.alignToHub(
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            this::updateHubShotSolutionAndGetAirtimeSeconds);
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
              stopGamePieceFlow();
            });
  }

  private Command openingHubShotAutoRoutine() {
    return timedHubShotAutoRoutine(START_AUTO_OPENING_SHOT_SECONDS);
  }

  private Command outpostStartShootAndLadderAutoRoutine() {
    Pose2d outpostStartPose = getAlliancePose(Constants.blueOutpost, Constants.redOutpost);
    Pose2d openingShotPose =
        getAlliancePose(BLUE_OUTPOST_OPENING_SHOT_POSE, RED_OUTPOST_OPENING_SHOT_POSE);

    return Commands.sequence(
            Commands.runOnce(() -> drive.setPose(outpostStartPose), drive),
            drive
                .alignToPose(openingShotPose)
                .withTimeout(START_AUTO_OUTPOST_TO_SHOOT_TIMEOUT_SECONDS),
            timedHubShotAutoRoutine(START_AUTO_OUTPOST_SHOOT_SECONDS),
            alignToLadderCommand().withTimeout(START_AUTO_LADDER_ALIGN_TIMEOUT_SECONDS))
        .finallyDo(
            () -> {
              shooter.setShotControlEnabled(false);
              stopGamePieceFlow();
            });
  }

  private Command timedHubShotAutoRoutine(double shotDurationSeconds) {
    return Commands.deadline(
        Commands.waitSeconds(shotDurationSeconds),
        drive.alignToHub(() -> 0.0, () -> 0.0, this::updateHubShotSolutionAndGetAirtimeSeconds),
        Commands.run(
                () -> {
                  updateHubShotSolutionAndGetAirtimeSeconds();
                  shooter.setShotControlEnabled(true);
                  applyBasicFeed(true);
                },
                shooter,
                intake,
                hopper,
                indexers)
            .beforeStarting(() -> shooter.setManualHoodOverrideEnabled(false))
            .finallyDo(
                () -> {
                  shooter.setShotControlEnabled(false);
                  stopGamePieceFlow();
                }));
  }

  private Command trenchCollectAutoRoutineNew() {
    return Commands.sequence(
        drive.autoDriveUnderTrenchCommand(),
        Commands.runOnce(
            () -> {
              intake.setTargetIntakeSpeed(BASIC_COLLECT_INTAKE_SPEED);
              intake.updateIntake();
            },
            intake),
        drive.autoLoadMiddleCommand(),
        Commands.runOnce(intake::stopIntake, intake),
        drive.autoDriveUnderTrenchCommand());
  }

  private Command trenchCollectAutoRoutine() {
    return Commands.deadline(
        drive.autoDriveUnderTrenchCommand().withTimeout(START_AUTO_TRENCH_TIMEOUT_SECONDS),
        basicCollectWhileHeldCommand(true));
  }

  private Command outpostCollectAutoRoutine() {
    return Commands.deadline(
        drive.driveToOutpostCommand().withTimeout(START_AUTO_OUTPOST_TIMEOUT_SECONDS),
        basicCollectWhileHeldCommand(true));
  }

  public Command alignToOutpost() {
    return drive.alignToOutpost(
        () -> -driverController.getLeftX(), () -> -driverController.getLeftY());
  }

  public Command alignToLadderCommand() {
    return drive.alignToPose(getAlliancePose(BLUE_LADDER_ALIGN_POSE, RED_LADDER_ALIGN_POSE));
  }

  public Command parkAtLadderL1Command() {
    return alignToLadderCommand();
  }

  public Command alignToDepotCommand() {
    return drive.alignToPose(getAlliancePose(BLUE_DEPOT_ALIGN_POSE, RED_DEPOT_ALIGN_POSE));
  }

  private Pose2d getAlliancePose(Pose2d bluePose, Pose2d redPose) {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? redPose : bluePose;
  }

  private Command runShooterDemandWhileHeldCommand() {
    return Commands.runEnd(
        () -> setShooterDemandFromTriggerThrottle(driverController.getRightTriggerAxis()),
        () -> setShooterDemandFromTriggerThrottle(0.0));
  }

  private Command runIntakePivotWhileHeldCommand(double speed) {
    return Commands.runEnd(
        () -> intake.setIntakePivotSpeed(speed), intake::stopIntakePivot, intake);
  }

  private Command runHopperExtensionWhileHeldCommand(double speed) {
    return Commands.runEnd(
        () -> hopper.setHopperExtensionSpeed(speed), hopper::stopHopperExtension, hopper);
  }

  private Command basicCollectWhileHeldCommand(boolean runIndexers) {
    return Commands.startEnd(
        () -> applyBasicCollect(runIndexers), this::stopGamePieceFlow, intake, hopper, indexers);
  }

  private Command basicReverseWhileHeldCommand() {
    return Commands.startEnd(
        this::applyBasicReverse, this::stopGamePieceFlow, intake, hopper, indexers);
  }

  private Command runManualFeedAndIndexersWhileHeldCommand() {
    return Commands.startEnd(
            () -> applyBasicFeed(false),
            () -> {
              intake.stopIntake();
              hopper.stopAgitator();
            },
            intake,
            hopper)
        .alongWith(runIndexersWhileHeldCommand());
  }

  private Command runIndexersWhileHeldCommand() {
    return Commands.runEnd(
            () -> {
              boolean allowIndexer =
                  ShooterFeedInterlock.shouldRunIndexerDuringManualFeed(
                      true, shooterDemandFromAlign, shooter.isHubShotSolutionFeasible());
              if (!allowIndexer) {
                indexers.stopIndexers();
                Logger.recordOutput("GamePieceControl/Mode", "MANUAL_FEED_INTERLOCKED");
                return;
              }

              boolean inBottomBreakaway =
                  Timer.getFPGATimestamp() < bottomIndexerBreakawayUntilTimestampSeconds;
              double bottomOutput =
                  inBottomBreakaway ? BOTTOM_INDEXER_BREAKAWAY_SPEED : BOTTOM_INDEXER_HOLD_SPEED;
              double topOutput = inBottomBreakaway ? 0.0 : TOP_INDEXER_MANUAL_FEED_SPEED;
              indexers.setManualOutputs(topOutput, bottomOutput);
            },
            indexers::stopIndexers,
            indexers)
        .beforeStarting(
            () ->
                bottomIndexerBreakawayUntilTimestampSeconds =
                    Timer.getFPGATimestamp() + BOTTOM_INDEXER_BREAKAWAY_SECONDS);
  }

  private void applyBasicCollect(boolean runIndexers) {
    intake.setTargetIntakeSpeed(BASIC_COLLECT_INTAKE_SPEED);
    hopper.setTargetAgitatorSpeed(BASIC_COLLECT_AGITATOR_SPEED);
    intake.updateIntake();
    hopper.updateAgitator();
    if (runIndexers) {
      indexers.setTargetIndexerSpeed(BASIC_COLLECT_INDEXER_SPEED);
      indexers.updateIndexers();
      Logger.recordOutput("GamePieceControl/Mode", "COLLECT");
    } else {
      indexers.stopIndexers();
      Logger.recordOutput("GamePieceControl/Mode", "COLLECT_NO_INDEXER");
    }
  }

  private void applyBasicFeed(boolean runIndexers) {
    intake.stopIntake();
    hopper.setTargetAgitatorSpeed(BASIC_FEED_AGITATOR_SPEED);
    hopper.updateAgitator();
    if (runIndexers) {
      boolean feedAllowed =
          ShooterFeedInterlock.shouldAdvanceToShooter(
              true,
              shooter.isReadyToFire(),
              true /* Sensorless path: assume staged piece is present when feed is requested. */);
      if (feedAllowed) {
        indexers.setTargetIndexerSpeed(BASIC_FEED_INDEXER_SPEED);
        indexers.updateIndexers();
        Logger.recordOutput("GamePieceControl/Mode", "FEED");
      } else {
        indexers.stopIndexers();
        Logger.recordOutput("GamePieceControl/Mode", "FEED_INTERLOCKED");
      }
    } else {
      indexers.stopIndexers();
      Logger.recordOutput("GamePieceControl/Mode", "MANUAL_FEED");
    }
  }

  private void applyBasicReverse() {
    intake.setTargetIntakeSpeed(BASIC_REVERSE_SPEED);
    hopper.setTargetAgitatorSpeed(BASIC_REVERSE_SPEED);
    indexers.setTargetIndexerSpeed(BASIC_REVERSE_SPEED);
    intake.updateIntake();
    hopper.updateAgitator();
    indexers.updateIndexers();
    Logger.recordOutput("GamePieceControl/Mode", "REVERSE");
  }

  private void stopGamePieceFlow() {
    intake.stopIntake();
    hopper.stopAgitator();
    indexers.stopIndexers();
    Logger.recordOutput("GamePieceControl/Mode", "IDLE");
  }

  private void setShooterDemandFromAlign(boolean enabled) {
    if (enabled) {
      // Re-enable automatic hood control whenever hub auto-align is engaged.
      shooter.setManualHoodOverrideEnabled(false);
    }
    shooterDemandFromAlign = enabled;
    refreshShooterControlDemand();
  }

  private void setShooterDemandFromTriggerThrottle(double throttle) {
    shooterDemandFromTriggerThrottle = MathUtil.clamp(throttle, 0.0, 1.0);
    refreshShooterControlDemand();
  }

  private void refreshShooterControlDemand() {
    boolean shooterDemandFromTrigger = shooterDemandFromTriggerThrottle > SHOOTER_TRIGGER_DEADBAND;
    boolean shooterDemandEnabled = shooterDemandFromAlign || shooterDemandFromTrigger;
    double shooterThrottleScale =
        shooterDemandEnabled
            ? (shooterDemandFromAlign ? 1.0 : shooterDemandFromTriggerThrottle)
            : 1.0;
    shooter.setOperatorWheelThrottleScale(shooterThrottleScale);
    shooter.setShotControlEnabled(shooterDemandEnabled);
  }

  public void periodic() {
    updateHubShotSolutionAndGetAirtimeSeconds();
    logRobotModelComponentPoses();
    updateEndgameWindowState();
    updateDriverRumbleOutput(Timer.getFPGATimestamp());
  }

  private void updateEndgameWindowState() {
    double matchTimeSeconds = DriverStation.getMatchTime();
    boolean endgameWindowActive =
        DriverStation.isTeleopEnabled()
            && matchTimeSeconds >= 0.0
            && matchTimeSeconds <= ENDGAME_WINDOW_START_SECONDS;

    if (endgameWindowActive && !endgameWindowLatched) {
      endgameRumbleUntilTimestampSeconds = Timer.getFPGATimestamp() + ENDGAME_RUMBLE_DURATION_SECS;
    }

    if (!DriverStation.isTeleopEnabled()) {
      endgameWindowLatched = false;
      endgameRumbleUntilTimestampSeconds = 0.0;
    } else {
      endgameWindowLatched = endgameWindowActive;
    }

    SmartDashboard.putBoolean("Match/EndgameWindowActive", endgameWindowActive);
    SmartDashboard.putNumber("Match/TimeRemainingSeconds", matchTimeSeconds);
    Logger.recordOutput("Match/EndgameWindowActive", endgameWindowActive);
    Logger.recordOutput("Match/TimeRemainingSeconds", matchTimeSeconds);
  }

  private void updateDriverRumbleOutput(double nowSeconds) {
    double rumbleStrength = 0.0;
    if (nowSeconds < endgameRumbleUntilTimestampSeconds) {
      rumbleStrength = Math.max(rumbleStrength, ENDGAME_RUMBLE_STRENGTH);
    }
    if (Constants.currentMode == Constants.Mode.SIM && nowSeconds < rumbleUntilTimestampSeconds) {
      rumbleStrength = Math.max(rumbleStrength, WALL_RUMBLE_STRENGTH);
    }

    driverController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, rumbleStrength);
    Logger.recordOutput("DriverFeedback/RumbleStrength", rumbleStrength);
    Logger.recordOutput(
        "DriverFeedback/EndgameRumbleActive", nowSeconds < endgameRumbleUntilTimestampSeconds);
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
    CommandScheduler.getInstance().schedule(activeAutoAssistCommand);
  }

  private void cancelAutoAssist() {
    if (activeAutoAssistCommand != null) {
      activeAutoAssistCommand.cancel();
      activeAutoAssistCommand = null;
    }
  }

  public void updateHubShotSolution() {
    updateHubShotSolutionAndGetAirtimeSeconds();
  }

  public void logRobotModelComponentPoses() {
    Logger.recordOutput(ROBOT_COMPONENTS_LOG_KEY, getRobotRelativeComponentPoses());
  }

  private double updateHubShotSolutionAndGetAirtimeSeconds() {
    shooter.updateHubShotSolution(
        drive.getPose(),
        drive.getAllianceHubPose(),
        drive.getFieldRelativeVelocityMetersPerSecond());
    return shooter.getHubAirtimeSeconds();
  }

  private Pose3d[] getRobotRelativeComponentPoses() {
    Pose3d[] componentPoses = new Pose3d[ROBOT_COMPONENT_COUNT];

    double intakePivotNormalized = intake.getIntakePivotMeasuredPositionNormalized();
    Rotation2d intakePivotPitch =
        Rotation2d.fromDegrees(
            MathUtil.interpolate(
                INTAKE_PIVOT_RETRACTED_ANGLE.getDegrees(),
                INTAKE_PIVOT_EXTENDED_ANGLE.getDegrees(),
                intakePivotNormalized));
    componentPoses[COMPONENT_INDEX_INTAKE_PIVOT] =
        new Pose3d(
            INTAKE_PIVOT_ORIGIN_ON_ROBOT, new Rotation3d(0.0, intakePivotPitch.getRadians(), 0.0));

    double hopperExtensionNormalized = hopper.getHopperExtensionMeasuredPositionNormalized();
    Translation3d hopperPosition =
        HOPPER_EXTENSION_RETRACTED_ORIGIN_ON_ROBOT.plus(
            HOPPER_EXTENSION_AXIS_ON_ROBOT.times(hopperExtensionNormalized));
    componentPoses[COMPONENT_INDEX_HOPPER_EXTENSION] = new Pose3d(hopperPosition, new Rotation3d());

    Rotation2d hoodPitch =
        shooter
            .getMeasuredHoodAngle()
            .minus(ShooterConstants.minHoodAngleFromFloor)
            .plus(SHOOTER_HOOD_MODEL_PITCH_OFFSET);
    componentPoses[COMPONENT_INDEX_SHOOTER_HOOD] =
        new Pose3d(SHOOTER_HOOD_PIVOT_ON_ROBOT, new Rotation3d(0.0, hoodPitch.getRadians(), 0.0));

    return componentPoses;
  }

  public void onDisabledInit() {
    shooterDemandFromAlign = false;
    shooterDemandFromTriggerThrottle = 0.0;
    refreshShooterControlDemand();
    stopGamePieceFlow();
    cancelAutoAssist();
    endgameWindowLatched = false;
    endgameRumbleUntilTimestampSeconds = 0.0;
    resetSimulationField();
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM || driveSimulation == null) {
      return;
    }

    SimulatedArena arena = SimulatedArena.getInstance();
    shooter.resetSimulationState();
    intake.resetSimulationState();
    hopper.resetSimulationState();
    indexers.resetSimulationState();
    drive.setPose(SIM_START_POSE);
    arena.resetFieldForAuto();
    previousSimFieldSpeeds = null;
    rumbleUntilTimestampSeconds = 0.0;
    endgameRumbleUntilTimestampSeconds = 0.0;
    simulatedShooterShotsLaunched = 0;
    simulatedShooterHubHits = 0;
    simulatedShooterShotsActiveHub = 0;
    simulatedShooterShotsInactiveHub = 0;
    driverController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
    Logger.recordOutput("Shooter/Simulation/ShotsLaunched", simulatedShooterShotsLaunched);
    Logger.recordOutput("Shooter/Simulation/HubHits", simulatedShooterHubHits);
    Logger.recordOutput("Shooter/Simulation/ShotsAtActiveHub", simulatedShooterShotsActiveHub);
    Logger.recordOutput("Shooter/Simulation/ShotsAtInactiveHub", simulatedShooterShotsInactiveHub);
    Logger.recordOutput("Shooter/Simulation/LastShotHubState", "UNKNOWN");
    Logger.recordOutput("Shooter/Simulation/ShotTrajectory", new Pose3d[] {});
    Logger.recordOutput("Shooter/Simulation/ActiveFuelProjectiles", new Pose3d[] {});
    Logger.recordOutput("Shooter/Simulation/LastLaunchSpeedMetersPerSec", 0.0);
    Logger.recordOutput("Shooter/Simulation/LastLaunchAngleDegrees", 0.0);
    Logger.recordOutput("Shooter/Simulation/LastLaunchYawDegrees", 0.0);
    Logger.recordOutput("Shooter/Simulation/LastLaunchPose3d", new Pose3d());
    Logger.recordOutput("GamePieceSimulation/Transfers/LastEvent", "Reset");
    Logger.recordOutput("GamePieceSimulation/Transfers/QueuedCount", 0);
    Logger.recordOutput("GamePieceSimulation/Stage/IntakeOccupied", false);
    Logger.recordOutput("GamePieceSimulation/Stage/HopperOccupied", false);
    Logger.recordOutput("GamePieceSimulation/Stage/ShooterOccupied", false);
    Logger.recordOutput("GamePieceSimulation/EjectPose", new Pose3d());
    Logger.recordOutput("GamePieceSimulation/MapleIntakeRunning", false);
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM || driveSimulation == null) {
      return;
    }

    SimulatedArena arena = SimulatedArena.getInstance();
    arena.simulationPeriodic();
    Pose2d robotPose = driveSimulation.getSimulatedDriveTrainPose();
    ChassisSpeeds simFieldSpeeds =
        driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative();
    maybeLaunchSimulatedFuel(robotPose, simFieldSpeeds);
    updateCollisionRumble(robotPose, simFieldSpeeds);
    HumpPoseSample humpPoseSample = sampleHumpPose(robotPose);
    Logger.recordOutput("FieldSimulation/RobotPose", robotPose);
    Logger.recordOutput(
        "FieldSimulation/RobotPose3d", getSimulatedRobotPose3d(robotPose, humpPoseSample));
    Logger.recordOutput(
        "FieldSimulation/RobotParts/SwerveModules",
        getSimulatedModulePoses(robotPose, humpPoseSample));
    Logger.recordOutput("FieldSimulation/GamePieces/Fuel", arena.getGamePiecesArrayByType("Fuel"));
    Logger.recordOutput("FieldSimulation/GamePieces/Note", arena.getGamePiecesArrayByType("Note"));
    Logger.recordOutput(
        "FieldSimulation/GamePieces/Coral", arena.getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/GamePieces/Algae", arena.getGamePiecesArrayByType("Algae"));
    Logger.recordOutput(
        "Shooter/Simulation/ActiveFuelProjectiles", getActiveFuelProjectilePoses(arena));
    Logger.recordOutput("Shooter/Simulation/ShotsLaunched", simulatedShooterShotsLaunched);
    Logger.recordOutput("Shooter/Simulation/HubHits", simulatedShooterHubHits);
  }

  private double getShooterFeedRateRatioForSimulation() {
    if (BASIC_FEED_INDEXER_SPEED <= 1e-6) {
      return 0.0;
    }
    return MathUtil.clamp(indexers.getAverageAppliedOutput() / BASIC_FEED_INDEXER_SPEED, 0.0, 1.0);
  }

  private void maybeLaunchSimulatedFuel(Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {
    double shooterFeedRateRatio = getShooterFeedRateRatioForSimulation();
    Logger.recordOutput("Shooter/Simulation/FeedRateRatio", shooterFeedRateRatio);
    if (!shooter.shouldTriggerSimulatedShot(Timer.getFPGATimestamp(), shooterFeedRateRatio)) {
      return;
    }

    Rotation2d shooterFacing = robotPose.getRotation().plus(ShooterConstants.shooterFacingOffset);
    Rotation2d launchPitch = shooter.getMeasuredHoodAngle();
    double launchSpeedMetersPerSec =
        MathUtil.clamp(
            shooter.getEstimatedLaunchSpeedFromMeasuredWheelsMetersPerSec(),
            ShooterConstants.minLaunchSpeedMetersPerSec,
            ShooterConstants.maxLaunchSpeedMetersPerSec);
    Pose2d targetHubPose = drive.getAllianceHubPose();
    GameStateSubsystem.HubState shotHubState = gameState.getHubState();

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
    if (shotHubState == GameStateSubsystem.HubState.ACTIVE) {
      simulatedShooterShotsActiveHub++;
    } else {
      simulatedShooterShotsInactiveHub++;
    }

    Logger.recordOutput("Shooter/Simulation/ShotsLaunched", simulatedShooterShotsLaunched);
    Logger.recordOutput("Shooter/Simulation/LastLaunchSpeedMetersPerSec", launchSpeedMetersPerSec);
    Logger.recordOutput("Shooter/Simulation/LastLaunchAngleDegrees", launchPitch.getDegrees());
    Logger.recordOutput("Shooter/Simulation/LastLaunchYawDegrees", shooterFacing.getDegrees());
    Logger.recordOutput("Shooter/Simulation/ShotsAtActiveHub", simulatedShooterShotsActiveHub);
    Logger.recordOutput("Shooter/Simulation/ShotsAtInactiveHub", simulatedShooterShotsInactiveHub);
    Logger.recordOutput("Shooter/Simulation/LastShotHubState", shotHubState.name());
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
