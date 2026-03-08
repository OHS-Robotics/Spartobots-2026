// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.endgame.Endgame;
import frc.robot.subsystems.endgame.SimpleEndgame;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.SimpleIndexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.SimpleIntake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterBallistics;
import frc.robot.subsystems.shooter.SimpleShooter;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureGoal;
import frc.robot.subsystems.superstructure.SuperstructureStatus;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static final double INTENT_TRIGGER_THRESHOLD = 0.5;
  private static final Pose2d SIM_START_POSE = FieldTargets.AUTO_START_CENTER.bluePose();
  private static final double HUB_TOP_LENGTH_Y_METERS = Units.inchesToMeters(47.0);
  private static final double HUB_RAMP_LENGTH_Y_METERS = Units.inchesToMeters(73.0);
  private static final double HUB_WIDTH_X_METERS = Units.inchesToMeters(47.0);
  private static final double HUMP_PEAK_HEIGHT_METERS = Units.inchesToMeters(7.0);
  private static final double HUMP_X_CLEARANCE_MARGIN_METERS = 0.15;
  private static final double ROBOT_BODY_BASE_HEIGHT_METERS = 0.12;
  private static final double MODULE_HEIGHT_ABOVE_GROUND_METERS = 0.05;
  private static final HumpPoseSample FLAT_GROUND_SAMPLE = new HumpPoseSample(0.0, 0.0);

  // Subsystems
  private final Drive drive;
  private final Intake intake = new SimpleIntake();
  private final Indexer indexer = new SimpleIndexer();
  private final Shooter shooter = new SimpleShooter();
  private final Endgame endgame = new SimpleEndgame();
  private final ShooterBallistics shooterBallistics = new ShooterBallistics();
  private final Superstructure superstructure;

  // Controller
  public final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private final Vision vision;
  private SwerveDriveSimulation driveSimulation = null;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
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
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0),
                new VisionIOPhotonVision(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1));
        break;
    }

    superstructure =
        new Superstructure(drive, intake, indexer, shooter, endgame, shooterBallistics);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

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
    drive.setDefaultCommand(rawDriveCommand());

    // Driver intent bindings
    controller.leftTrigger().whileTrue(acquireIntent());
    controller.rightTrigger().whileTrue(autoFaceAndScoreIntent());
    controller.leftBumper().whileTrue(outpostFeedIntent());
    controller.b().onTrue(cancelRecoverIntent());
    controller.y().whileTrue(quickParkIntent());
    controller.x().whileTrue(manualOverrideIntent());

    if (Constants.currentMode == Constants.Mode.SIM) {
      controller.start().onTrue(Commands.runOnce(this::resetSimulationField).ignoringDisable(true));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return autoChooser.get();
    return drive.getAutonomousCommand();
  }

  public void logOperatorFeedback(
      MatchStateProvider.MatchState matchState, Command autonomousCommand) {
    SuperstructureStatus superstructureStatus = superstructure.getStatus();
    boolean poseTrusted = drive.isPoseTrusted();
    OperatorMode selectedMode = getSelectedMode(autonomousCommand);
    boolean shotReady = isShotReady(superstructureStatus, poseTrusted);

    Logger.recordOutput("OperatorFeedback/ActiveHub", matchState.currentActiveHub().name());
    Logger.recordOutput("OperatorFeedback/SelectedMode", selectedMode.name());
    Logger.recordOutput("OperatorFeedback/MagazineCount", getMagazineCount(superstructureStatus));
    Logger.recordOutput("OperatorFeedback/ShotReady", shotReady);
    Logger.recordOutput(
        "OperatorFeedback/ShotReadyReason", getShotReadyReason(superstructureStatus, poseTrusted));
    Logger.recordOutput("OperatorFeedback/PoseTrusted", poseTrusted);
    Logger.recordOutput("OperatorFeedback/PoseConfidence", getPoseConfidenceLabel(poseTrusted));
    Logger.recordOutput(
        "OperatorFeedback/CurrentAutoAction", getCurrentAutoAction(autonomousCommand));
  }

  private Command rawDriveCommand() {
    return DriveCommands.joystickDrive(
        drive,
        () -> controller.getLeftY(),
        () -> controller.getLeftX(),
        () -> -controller.getRightX());
  }

  private Command acquireIntent() {
    return maintainGoal(this::selectAcquireGoal);
  }

  private Command autoFaceAndScoreIntent() {
    return driverAssistIntent(
        this::selectAutoFaceAndScoreGoal,
        this::getAssistDriveX,
        this::getAssistDriveY,
        this::getSuperstructureTargetHeading);
  }

  private Command outpostFeedIntent() {
    return driverAssistIntent(
        () -> new SuperstructureGoal.OutpostAlign(),
        this::getOutpostAssistDriveX,
        this::getOutpostAssistDriveY,
        this::getSuperstructureTargetHeading);
  }

  private Command quickParkIntent() {
    return driverAssistIntent(
        () ->
            new SuperstructureGoal.Endgame(
                SuperstructureGoal.EndgamePhase.PREP, selectQuickParkZone()),
        this::getAssistDriveX,
        this::getAssistDriveY,
        this::getQuickParkHeading);
  }

  private Command cancelRecoverIntent() {
    return Commands.parallel(
        Commands.runOnce(superstructure::clearGoal, superstructure),
        Commands.runOnce(drive::stop, drive));
  }

  private Command manualOverrideIntent() {
    return Commands.parallel(
        Commands.run(superstructure::clearGoal, superstructure), rawDriveCommand());
  }

  private Command driverAssistIntent(
      Supplier<SuperstructureGoal> goalSupplier,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> targetHeadingSupplier) {
    return Commands.parallel(
        maintainGoal(goalSupplier), drive.faceTarget(xSupplier, ySupplier, targetHeadingSupplier));
  }

  private Command maintainGoal(Supplier<SuperstructureGoal> goalSupplier) {
    return Commands.run(() -> superstructure.setGoal(goalSupplier.get()), superstructure)
        .finallyDo((interrupted) -> superstructure.clearGoal());
  }

  private SuperstructureGoal selectAcquireGoal() {
    return selectAcquireGoal(
        drive.getPose(),
        superstructure.hasGamePiece(),
        TargetSelector.getIntakeZone(TargetSelector.IntakeZoneSelection.ALLIANCE_DEPOT),
        TargetSelector.getIntakeZone(TargetSelector.IntakeZoneSelection.NEUTRAL_FLOOR));
  }

  static SuperstructureGoal selectAcquireGoal(
      Pose2d robotPose,
      boolean hasGamePiece,
      FieldTargets.FieldZone depotZone,
      FieldTargets.FieldZone floorZone) {
    if (hasGamePiece) {
      return new SuperstructureGoal.Stow();
    }
    if (depotZone.contains(robotPose.getTranslation())) {
      return new SuperstructureGoal.IntakeDepot(SuperstructureGoal.IntakePhase.SETTLE);
    }
    if (floorZone.contains(robotPose.getTranslation())) {
      return new SuperstructureGoal.IntakeFloor(SuperstructureGoal.IntakePhase.SETTLE);
    }
    return distanceToZoneCenter(robotPose, depotZone) <= distanceToZoneCenter(robotPose, floorZone)
        ? new SuperstructureGoal.IntakeDepot(SuperstructureGoal.IntakePhase.SETTLE)
        : new SuperstructureGoal.IntakeFloor(SuperstructureGoal.IntakePhase.SETTLE);
  }

  private SuperstructureGoal selectAutoFaceAndScoreGoal() {
    return new SuperstructureGoal.HubShot(
        superstructure.hasGamePiece()
            ? SuperstructureGoal.HubShotPhase.FIRE
            : SuperstructureGoal.HubShotPhase.AIM);
  }

  private TargetSelector.ParkZoneSelection selectQuickParkZone() {
    return selectQuickParkZone(
        drive.getPose(),
        TargetSelector.getParkZone(TargetSelector.ParkZoneSelection.ALLIANCE_LOWER),
        TargetSelector.getParkZone(TargetSelector.ParkZoneSelection.ALLIANCE_UPPER));
  }

  static TargetSelector.ParkZoneSelection selectQuickParkZone(
      Pose2d robotPose, FieldTargets.FieldZone lowerZone, FieldTargets.FieldZone upperZone) {
    if (lowerZone.contains(robotPose.getTranslation())) {
      return TargetSelector.ParkZoneSelection.ALLIANCE_LOWER;
    }
    if (upperZone.contains(robotPose.getTranslation())) {
      return TargetSelector.ParkZoneSelection.ALLIANCE_UPPER;
    }
    return distanceToZoneCenter(robotPose, lowerZone) <= distanceToZoneCenter(robotPose, upperZone)
        ? TargetSelector.ParkZoneSelection.ALLIANCE_LOWER
        : TargetSelector.ParkZoneSelection.ALLIANCE_UPPER;
  }

  private Rotation2d getSuperstructureTargetHeading() {
    return superstructure.getStatus().targetHeading();
  }

  private Rotation2d getQuickParkHeading() {
    return computeHeadingToZoneCenter(
        drive.getPose(), TargetSelector.getParkZone(selectQuickParkZone()));
  }

  static Rotation2d computeHeadingToZoneCenter(Pose2d robotPose, FieldTargets.FieldZone zone) {
    Translation2d toZoneCenter = zone.center().minus(robotPose.getTranslation());
    if (toZoneCenter.getNorm() <= 1e-9) {
      return robotPose.getRotation();
    }
    return new Rotation2d(Math.atan2(toZoneCenter.getY(), toZoneCenter.getX()));
  }

  private static double distanceToZoneCenter(Pose2d robotPose, FieldTargets.FieldZone zone) {
    return robotPose.getTranslation().getDistance(zone.center());
  }

  private double getAssistDriveX() {
    return -controller.getLeftY();
  }

  private double getAssistDriveY() {
    return -controller.getLeftX();
  }

  private double getOutpostAssistDriveX() {
    return -controller.getLeftX();
  }

  private double getOutpostAssistDriveY() {
    return -controller.getLeftY();
  }

  private OperatorMode getSelectedMode(Command autonomousCommand) {
    if (DriverStation.isAutonomousEnabled()
        || (autonomousCommand != null && autonomousCommand.isScheduled())) {
      return OperatorMode.AUTONOMOUS;
    }

    XboxController hid = controller.getHID();
    if (hid.getXButton()) {
      return OperatorMode.MANUAL_OVERRIDE;
    }
    if (hid.getYButton()) {
      return OperatorMode.QUICK_PARK;
    }
    if (hid.getLeftBumperButton()) {
      return OperatorMode.OUTPOST_FEED;
    }
    if (hid.getRightTriggerAxis() > INTENT_TRIGGER_THRESHOLD) {
      return OperatorMode.AUTO_FACE_AND_SCORE;
    }
    if (hid.getLeftTriggerAxis() > INTENT_TRIGGER_THRESHOLD) {
      return OperatorMode.ACQUIRE;
    }
    return OperatorMode.MANUAL_DRIVE;
  }

  private int getMagazineCount(SuperstructureStatus superstructureStatus) {
    return getMagazineCount(superstructureStatus.hasGamePiece());
  }

  static int getMagazineCount(boolean hasGamePiece) {
    return hasGamePiece ? 1 : 0;
  }

  private String getCurrentAutoAction(Command autonomousCommand) {
    if (autonomousCommand != null && autonomousCommand.isScheduled()) {
      return autonomousCommand.getName();
    }
    return getCurrentAutoAction(superstructure.getStatus().activeGoal());
  }

  static String getCurrentAutoAction(SuperstructureGoal activeGoal) {
    if (activeGoal instanceof SuperstructureGoal.IntakeDepot intakeDepot) {
      return "ACQUIRE_DEPOT_" + intakeDepot.phase().name();
    }
    if (activeGoal instanceof SuperstructureGoal.IntakeFloor intakeFloor) {
      return "ACQUIRE_FLOOR_" + intakeFloor.phase().name();
    }
    if (activeGoal instanceof SuperstructureGoal.HubShot hubShot) {
      return "AUTO_FACE_AND_SCORE_" + hubShot.phase().name();
    }
    if (activeGoal instanceof SuperstructureGoal.OutpostAlign) {
      return "OUTPOST_FEED_ALIGN";
    }
    if (activeGoal instanceof SuperstructureGoal.Eject eject) {
      return "RECOVER_" + eject.phase().name();
    }
    if (activeGoal instanceof SuperstructureGoal.Endgame endgameGoal) {
      return "QUICK_PARK_" + endgameGoal.phase().name();
    }
    return "IDLE";
  }

  static boolean isShotReady(SuperstructureStatus superstructureStatus, boolean poseTrusted) {
    return "READY".equals(getShotReadyReason(superstructureStatus, poseTrusted));
  }

  static String getShotReadyReason(SuperstructureStatus superstructureStatus, boolean poseTrusted) {
    if (!(superstructureStatus.activeGoal() instanceof SuperstructureGoal.HubShot)) {
      return "MODE_NOT_SCORING";
    }
    if (!superstructureStatus.hasGamePiece()) {
      return "NO_GAME_PIECE";
    }
    if (!poseTrusted) {
      return "POSE_UNTRUSTED";
    }
    if (!superstructureStatus.shooterReady()) {
      return "SHOOTER_NOT_READY";
    }
    if (!superstructureStatus.driveAligned()) {
      return "DRIVE_NOT_ALIGNED";
    }
    if (!superstructureStatus.atGoal()) {
      return "WAITING_FOR_FIRE_WINDOW";
    }
    return "READY";
  }

  static String getPoseConfidenceLabel(boolean poseTrusted) {
    return poseTrusted ? "TRUSTED" : "UNTRUSTED";
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM || driveSimulation == null) {
      return;
    }

    drive.setPose(SIM_START_POSE);
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM || driveSimulation == null) {
      return;
    }

    SimulatedArena.getInstance().simulationPeriodic();
    Pose2d robotPose = driveSimulation.getSimulatedDriveTrainPose();
    HumpPoseSample humpPoseSample = sampleHumpPose(robotPose);
    Logger.recordOutput("FieldSimulation/RobotPose", robotPose);
    Logger.recordOutput(
        "FieldSimulation/RobotPose3d", getSimulatedRobotPose3d(robotPose, humpPoseSample));
    Logger.recordOutput(
        "FieldSimulation/RobotParts/SwerveModules",
        getSimulatedModulePoses(robotPose, humpPoseSample));
    Logger.recordOutput(
        "FieldSimulation/GamePieces/Fuel",
        SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
    Logger.recordOutput(
        "FieldSimulation/GamePieces/Note",
        SimulatedArena.getInstance().getGamePiecesArrayByType("Note"));
    Logger.recordOutput(
        "FieldSimulation/GamePieces/Coral",
        SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/GamePieces/Algae",
        SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }

  private Pose3d getSimulatedRobotPose3d(Pose2d robotPose, HumpPoseSample humpPoseSample) {
    return new Pose3d(
        new Translation3d(
            robotPose.getX(),
            robotPose.getY(),
            ROBOT_BODY_BASE_HEIGHT_METERS + humpPoseSample.heightMeters()),
        new Rotation3d(humpPoseSample.rollRadians(), 0.0, robotPose.getRotation().getRadians()));
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
                  MODULE_HEIGHT_ABOVE_GROUND_METERS + humpPoseSample.heightMeters()),
              new Rotation3d(humpPoseSample.rollRadians(), 0.0, moduleRotation.getRadians()));
    }
    return modulePoses;
  }

  private HumpPoseSample sampleHumpPose(Pose2d robotPose) {
    HumpPoseSample blueSample = sampleSingleHump(FieldTargets.HUB.bluePose().toPose2d(), robotPose);
    HumpPoseSample redSample = sampleSingleHump(FieldTargets.HUB.redPose().toPose2d(), robotPose);
    return blueSample.heightMeters() >= redSample.heightMeters() ? blueSample : redSample;
  }

  private HumpPoseSample sampleSingleHump(Pose2d hubPose, Pose2d robotPose) {
    double maxHumpXDistance =
        (HUB_WIDTH_X_METERS / 2.0)
            + (DriveConstants.trackWidth / 2.0)
            + HUMP_X_CLEARANCE_MARGIN_METERS;
    if (Math.abs(robotPose.getX() - hubPose.getX()) > maxHumpXDistance) {
      return FLAT_GROUND_SAMPLE;
    }

    double yOffsetFromHub = robotPose.getY() - hubPose.getY();
    double absYOffset = Math.abs(yOffsetFromHub);
    double halfTopLength = HUB_TOP_LENGTH_Y_METERS / 2.0;
    if (absYOffset > (halfTopLength + HUB_RAMP_LENGTH_Y_METERS)) {
      return FLAT_GROUND_SAMPLE;
    }

    if (absYOffset <= halfTopLength) {
      return new HumpPoseSample(HUMP_PEAK_HEIGHT_METERS, 0.0);
    }

    double rampTravelMeters = absYOffset - halfTopLength;
    double rampPercent =
        1.0 - MathUtil.clamp(rampTravelMeters / HUB_RAMP_LENGTH_Y_METERS, 0.0, 1.0);
    double heightMeters = HUMP_PEAK_HEIGHT_METERS * rampPercent;
    double maxRollRadians = Math.atan2(HUMP_PEAK_HEIGHT_METERS, HUB_RAMP_LENGTH_Y_METERS);
    double rollRadians = Math.copySign(maxRollRadians, -yOffsetFromHub);
    return new HumpPoseSample(heightMeters, rollRadians);
  }

  private record HumpPoseSample(double heightMeters, double rollRadians) {}

  private static enum OperatorMode {
    MANUAL_DRIVE,
    ACQUIRE,
    AUTO_FACE_AND_SCORE,
    OUTPOST_FEED,
    QUICK_PARK,
    MANUAL_OVERRIDE,
    AUTONOMOUS
  }
}
