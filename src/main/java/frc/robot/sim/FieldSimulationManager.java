package frc.robot.sim;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.game.GameStateSubsystem;
import frc.robot.operator.OperatorFeedbackController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.gamepiece.indexers.Indexers;
import frc.robot.subsystems.gamepiece.intake.Intake;
import frc.robot.subsystems.gamepiece.shooter.Shooter;
import frc.robot.subsystems.gamepiece.shooter.ShooterConstants;
import frc.robot.superstructure.gamepiece.GamePieceCoordinator;
import frc.robot.targeting.HubTargetingGeometry;
import frc.robot.util.NetworkTablesUtil;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

public class FieldSimulationManager {
  private static final Pose2d simStartPose = Constants.simulationStartPose;
  private static final Pose3d[] emptyPose3dArray = new Pose3d[] {};
  private static final String fuelGamePieceType = "Fuel";
  private static final double hubTopLengthYMeters = Units.inchesToMeters(47.0);
  private static final double hubRampLengthYMeters = Units.inchesToMeters(73.0);
  private static final double hubWidthXMeters = Units.inchesToMeters(47.0);
  private static final double humpPeakHeightMeters = Units.inchesToMeters(7.0);
  private static final double humpXClearanceMarginMeters = 0.15;
  private static final double simTerrainPoseSmoothingTimeConstantSecs = 0.10;
  private static final double fieldXMinMeters = 0.0;
  private static final double fieldXMaxMeters = 16.54105;
  private static final double fieldYMinMeters = 0.0;
  private static final double fieldYMaxMeters = 8.06926;
  private static final double wallImpactMarginMeters = 0.08;
  private static final double wallImpactMinApproachSpeedMps = 0.55;
  private static final double wallImpactAccelThresholdMps2 = 9.0;
  private static final double wallRumbleDurationSecs = 0.22;
  private static final double wallRumbleStrength = 1.0;
  private static final double robotBodyBaseHeightMeters = 0.0;
  private static final double moduleHeightAboveGroundMeters = 0.05;

  private final Drive drive;
  private final Shooter shooter;
  private final Intake intake;
  private final Indexers indexers;
  private final GameStateSubsystem gameState;
  private final GamePieceCoordinator gamePieceCoordinator;
  private final OperatorFeedbackController feedbackController;
  private final SwerveDriveSimulation driveSimulation;

  private ChassisSpeeds previousSimFieldSpeeds = null;
  private int simulatedShooterShotsLaunched = 0;
  private int simulatedShooterHubHits = 0;
  private int simulatedShooterShotsActiveHub = 0;
  private int simulatedShooterShotsInactiveHub = 0;
  private HumpPoseSample filteredHumpPoseSample = null;
  private double lastSimTerrainSampleTimestampSeconds = Double.NaN;

  public FieldSimulationManager(
      Drive drive,
      Shooter shooter,
      Intake intake,
      Indexers indexers,
      GameStateSubsystem gameState,
      GamePieceCoordinator gamePieceCoordinator,
      OperatorFeedbackController feedbackController,
      SwerveDriveSimulation driveSimulation) {
    this.drive = drive;
    this.shooter = shooter;
    this.intake = intake;
    this.indexers = indexers;
    this.gameState = gameState;
    this.gamePieceCoordinator = gamePieceCoordinator;
    this.feedbackController = feedbackController;
    this.driveSimulation = driveSimulation;
  }

  public void resetField() {
    if (Constants.currentMode != Constants.Mode.SIM || driveSimulation == null) {
      return;
    }

    SimulatedArena arena = SimulatedArena.getInstance();
    shooter.resetSimulationState();
    intake.resetSimulationState();
    indexers.resetSimulationState();
    drive.setPose(simStartPose);
    arena.resetFieldForAuto();
    previousSimFieldSpeeds = null;
    simulatedShooterShotsLaunched = 0;
    simulatedShooterHubHits = 0;
    simulatedShooterShotsActiveHub = 0;
    simulatedShooterShotsInactiveHub = 0;
    filteredHumpPoseSample = null;
    lastSimTerrainSampleTimestampSeconds = Double.NaN;
    feedbackController.clearTransientState();

    Logger.recordOutput(
        NetworkTablesUtil.logPath("Simulation/Shooter/ShotsLaunched"),
        simulatedShooterShotsLaunched);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Simulation/Shooter/HubHits"), simulatedShooterHubHits);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Simulation/Shooter/ShotsAtActiveHub"),
        simulatedShooterShotsActiveHub);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Simulation/Shooter/ShotsAtInactiveHub"),
        simulatedShooterShotsInactiveHub);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Simulation/Shooter/LastShotHubState"), "UNKNOWN");
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Simulation/Shooter/ShotTrajectory"), emptyPose3dArray);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Simulation/Shooter/ActiveFuelProjectiles"), emptyPose3dArray);
    Pose3d[] fuelGamePiecePoses = getFuelGamePiecePoses(arena);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Simulation/Field/GamePieces/Fuel"), fuelGamePiecePoses);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Simulation/Shooter/LastLaunchSpeedMetersPerSec"), 0.0);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Simulation/Shooter/LastLaunchAngleDegrees"), 0.0);
    Logger.recordOutput(NetworkTablesUtil.logPath("Simulation/Shooter/LastLaunchYawDegrees"), 0.0);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Simulation/Shooter/LastLaunchPose3d"), new Pose3d());
  }

  public void update() {
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
    HumpPoseSample humpPoseSample =
        smoothHumpPoseSample(sampleHumpPose(robotPose), Timer.getFPGATimestamp());
    Logger.recordOutput(NetworkTablesUtil.logPath("Simulation/Field/RobotPose"), robotPose);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Simulation/Field/RobotPose3d"),
        getSimulatedRobotPose3d(robotPose, humpPoseSample));
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Simulation/Field/RobotParts/SwerveModules"),
        getSimulatedModulePoses(robotPose, humpPoseSample));
    Pose3d[] fuelGamePiecePoses = getFuelGamePiecePoses(arena);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Simulation/Field/GamePieces/Fuel"), fuelGamePiecePoses);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Simulation/Field/GamePieces/Note"),
        arena.getGamePiecesArrayByType("Note"));
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Simulation/Field/GamePieces/Coral"),
        arena.getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Simulation/Field/GamePieces/Algae"),
        arena.getGamePiecesArrayByType("Algae"));
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Simulation/Shooter/ActiveFuelProjectiles"),
        getActiveFuelProjectilePoses(arena));
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Simulation/Shooter/ShotsLaunched"),
        simulatedShooterShotsLaunched);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Simulation/Shooter/HubHits"), simulatedShooterHubHits);
  }

  private double getShooterFeedRateRatioForSimulation() {
    double basicFeedIndexerSpeed = gamePieceCoordinator.getBasicFeedIndexerSpeed();
    if (basicFeedIndexerSpeed <= 1e-6) {
      return 0.0;
    }
    return MathUtil.clamp(
        indexers.getAverageAppliedOutputMagnitude() / basicFeedIndexerSpeed, 0.0, 1.0);
  }

  private void maybeLaunchSimulatedFuel(Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {
    double shooterFeedRateRatio = getShooterFeedRateRatioForSimulation();
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Simulation/Shooter/FeedRateRatio"), shooterFeedRateRatio);
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
            getMapleSimShooterOffsetOnRobot(),
            fieldRelativeSpeeds,
            shooterFacing,
            Meters.of(shooter.getLaunchHeightMeters()),
            MetersPerSecond.of(launchSpeedMetersPerSec),
            Radians.of(launchPitch.getRadians()));

    projectile
        .withTargetPosition(
            () ->
                new Translation3d(
                    targetHubPose.getX(), targetHubPose.getY(), shooter.getHubAimHeightMeters()))
        .withTargetTolerance(
            new Translation3d(
                ShooterConstants.projectileTargetToleranceXYMeters,
                ShooterConstants.projectileTargetToleranceXYMeters,
                ShooterConstants.projectileTargetToleranceZMeters))
        .withHitTargetCallBack(
            () -> {
              simulatedShooterHubHits++;
              Logger.recordOutput(
                  NetworkTablesUtil.logPath("Simulation/Shooter/HubHits"), simulatedShooterHubHits);
            })
        .withProjectileTrajectoryDisplayCallBack(
            trajectory ->
                Logger.recordOutput(
                    NetworkTablesUtil.logPath("Simulation/Shooter/ShotTrajectory"),
                    trajectory.toArray(Pose3d[]::new)));

    SimulatedArena.getInstance().addGamePieceProjectile(projectile);
    simulatedShooterShotsLaunched++;
    if (shotHubState == GameStateSubsystem.HubState.ACTIVE) {
      simulatedShooterShotsActiveHub++;
    } else {
      simulatedShooterShotsInactiveHub++;
    }

    Logger.recordOutput(
        NetworkTablesUtil.logPath("Simulation/Shooter/ShotsLaunched"),
        simulatedShooterShotsLaunched);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Simulation/Shooter/LastLaunchSpeedMetersPerSec"),
        launchSpeedMetersPerSec);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Simulation/Shooter/LastLaunchAngleDegrees"),
        launchPitch.getDegrees());
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Simulation/Shooter/LastLaunchYawDegrees"),
        shooterFacing.getDegrees());
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Simulation/Shooter/ShotsAtActiveHub"),
        simulatedShooterShotsActiveHub);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Simulation/Shooter/ShotsAtInactiveHub"),
        simulatedShooterShotsInactiveHub);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Simulation/Shooter/LastShotHubState"), shotHubState.name());
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Simulation/Shooter/LastLaunchPose3d"),
        new Pose3d(
            getSimulatedLaunchOrigin(robotPose).getX(),
            getSimulatedLaunchOrigin(robotPose).getY(),
            shooter.getLaunchHeightMeters(),
            new Rotation3d(0.0, -launchPitch.getRadians(), shooterFacing.getRadians())));
  }

  private void updateCollisionRumble(Pose2d robotPose, ChassisSpeeds currentFieldSpeeds) {
    if (previousSimFieldSpeeds != null) {
      double deltaVx =
          currentFieldSpeeds.vxMetersPerSecond - previousSimFieldSpeeds.vxMetersPerSecond;
      double deltaVy =
          currentFieldSpeeds.vyMetersPerSecond - previousSimFieldSpeeds.vyMetersPerSecond;
      double translationalAccelerationMetersPerSecSquared = Math.hypot(deltaVx, deltaVy) / 0.02;

      boolean approachingWall = isApproachingFieldWall(robotPose, previousSimFieldSpeeds);
      boolean impactDetected =
          approachingWall
              && translationalAccelerationMetersPerSecSquared >= wallImpactAccelThresholdMps2;

      if (impactDetected) {
        feedbackController.triggerImpactRumble(wallRumbleDurationSecs, wallRumbleStrength);
      }

      Logger.recordOutput(
          NetworkTablesUtil.logPath("Simulation/Field/WallImpact/ApproachingWall"),
          approachingWall);
      Logger.recordOutput(
          NetworkTablesUtil.logPath("Simulation/Field/WallImpact/TranslationalAccelerationMps2"),
          translationalAccelerationMetersPerSecSquared);
      Logger.recordOutput(
          NetworkTablesUtil.logPath("Simulation/Field/WallImpact/Detected"), impactDetected);
      Logger.recordOutput(
          NetworkTablesUtil.logPath("Simulation/Field/WallImpact/RumbleStrength"),
          impactDetected ? wallRumbleStrength : 0.0);
    }

    previousSimFieldSpeeds = currentFieldSpeeds;
  }

  private Pose3d[] getActiveFuelProjectilePoses(SimulatedArena arena) {
    return arena.gamePieceLaunched().stream()
        .filter(projectile -> fuelGamePieceType.equals(projectile.getType()))
        .map(projectile -> projectile.getPose3d())
        .toArray(Pose3d[]::new);
  }

  static Pose3d[] getFuelGamePiecePoses(SimulatedArena arena) {
    return arena.getGamePiecesArrayByType(fuelGamePieceType);
  }

  static Translation2d getMapleSimShooterOffsetOnRobot() {
    // MapleSim rotates this offset by the shooter facing direction, so convert our normal
    // robot-frame muzzle offset into that frame before constructing the projectile.
    return ShooterConstants.shooterMuzzleOffsetOnRobot.rotateBy(
        Rotation2d.fromRadians(-ShooterConstants.shooterFacingOffset.getRadians()));
  }

  static Translation2d getSimulatedLaunchOrigin(Pose2d robotPose) {
    return HubTargetingGeometry.getLaunchOriginFieldPosition(robotPose);
  }

  private boolean isApproachingFieldWall(Pose2d robotPose, ChassisSpeeds fieldSpeeds) {
    double halfLength = DriveConstants.bumperLengthXMeters * 0.5;
    double halfWidth = DriveConstants.bumperWidthYMeters * 0.5;

    boolean nearMinX = robotPose.getX() <= fieldXMinMeters + halfLength + wallImpactMarginMeters;
    boolean nearMaxX = robotPose.getX() >= fieldXMaxMeters - halfLength - wallImpactMarginMeters;
    boolean nearMinY = robotPose.getY() <= fieldYMinMeters + halfWidth + wallImpactMarginMeters;
    boolean nearMaxY = robotPose.getY() >= fieldYMaxMeters - halfWidth - wallImpactMarginMeters;

    boolean approachingMinX =
        nearMinX && fieldSpeeds.vxMetersPerSecond < -wallImpactMinApproachSpeedMps;
    boolean approachingMaxX =
        nearMaxX && fieldSpeeds.vxMetersPerSecond > wallImpactMinApproachSpeedMps;
    boolean approachingMinY =
        nearMinY && fieldSpeeds.vyMetersPerSecond < -wallImpactMinApproachSpeedMps;
    boolean approachingMaxY =
        nearMaxY && fieldSpeeds.vyMetersPerSecond > wallImpactMinApproachSpeedMps;

    return approachingMinX || approachingMaxX || approachingMinY || approachingMaxY;
  }

  private Pose3d getSimulatedRobotPose3d(Pose2d robotPose, HumpPoseSample humpPoseSample) {
    return new Pose3d(
        new Translation3d(
            robotPose.getX(),
            robotPose.getY(),
            robotBodyBaseHeightMeters + humpPoseSample.heightMeters()),
        buildTerrainAlignedRotation(robotPose.getRotation(), humpPoseSample.surfaceNormal()));
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
                  moduleHeightAboveGroundMeters + humpPoseSample.moduleHeightsMeters()[i]),
              buildTerrainAlignedRotation(moduleRotation, humpPoseSample.surfaceNormal()));
    }
    return modulePoses;
  }

  private HumpPoseSample sampleHumpPose(Pose2d robotPose) {
    double[] moduleHeights = new double[DriveConstants.moduleTranslations.length];
    Translation2d[] moduleFieldPositions =
        new Translation2d[DriveConstants.moduleTranslations.length];
    for (int i = 0; i < moduleHeights.length; i++) {
      Translation2d moduleFieldPosition =
          robotPose
              .getTranslation()
              .plus(DriveConstants.moduleTranslations[i].rotateBy(robotPose.getRotation()));
      moduleFieldPositions[i] = moduleFieldPosition;
      moduleHeights[i] = sampleGroundHeightAt(moduleFieldPosition);
    }

    TerrainPlane plane = fitTerrainPlane(moduleFieldPositions, moduleHeights);
    double centerHeight =
        sampleTerrainHeightAt(
            plane, robotPose.getTranslation().getX(), robotPose.getTranslation().getY());
    return new HumpPoseSample(moduleHeights, centerHeight, plane.surfaceNormal());
  }

  private HumpPoseSample smoothHumpPoseSample(HumpPoseSample rawSample, double timestampSeconds) {
    if (filteredHumpPoseSample == null || Double.isNaN(lastSimTerrainSampleTimestampSeconds)) {
      filteredHumpPoseSample = rawSample;
      lastSimTerrainSampleTimestampSeconds = timestampSeconds;
      return filteredHumpPoseSample;
    }

    double dtSeconds = Math.max(0.0, timestampSeconds - lastSimTerrainSampleTimestampSeconds);
    double blend =
        dtSeconds <= 1e-9
            ? 1.0
            : 1.0 - Math.exp(-dtSeconds / simTerrainPoseSmoothingTimeConstantSecs);

    double[] smoothedModuleHeights = new double[rawSample.moduleHeightsMeters().length];
    for (int i = 0; i < smoothedModuleHeights.length; i++) {
      smoothedModuleHeights[i] =
          MathUtil.interpolate(
              filteredHumpPoseSample.moduleHeightsMeters()[i],
              rawSample.moduleHeightsMeters()[i],
              blend);
    }

    double smoothedHeight =
        MathUtil.interpolate(
            filteredHumpPoseSample.heightMeters(), rawSample.heightMeters(), blend);
    Translation3d smoothedNormal =
        normalizeVector(
            MathUtil.interpolate(
                filteredHumpPoseSample.surfaceNormal().getX(),
                rawSample.surfaceNormal().getX(),
                blend),
            MathUtil.interpolate(
                filteredHumpPoseSample.surfaceNormal().getY(),
                rawSample.surfaceNormal().getY(),
                blend),
            MathUtil.interpolate(
                filteredHumpPoseSample.surfaceNormal().getZ(),
                rawSample.surfaceNormal().getZ(),
                blend));

    filteredHumpPoseSample =
        new HumpPoseSample(smoothedModuleHeights, smoothedHeight, smoothedNormal);
    lastSimTerrainSampleTimestampSeconds = timestampSeconds;
    return filteredHumpPoseSample;
  }

  private TerrainPlane fitTerrainPlane(Translation2d[] samplePositions, double[] sampleHeights) {
    double meanX = 0.0;
    double meanY = 0.0;
    double meanZ = 0.0;
    for (int i = 0; i < sampleHeights.length; i++) {
      meanX += samplePositions[i].getX();
      meanY += samplePositions[i].getY();
      meanZ += sampleHeights[i];
    }
    meanX /= sampleHeights.length;
    meanY /= sampleHeights.length;
    meanZ /= sampleHeights.length;

    double sumDx2 = 0.0;
    double sumDy2 = 0.0;
    double sumDxDy = 0.0;
    double sumDxDz = 0.0;
    double sumDyDz = 0.0;
    for (int i = 0; i < sampleHeights.length; i++) {
      double dx = samplePositions[i].getX() - meanX;
      double dy = samplePositions[i].getY() - meanY;
      double dz = sampleHeights[i] - meanZ;
      sumDx2 += dx * dx;
      sumDy2 += dy * dy;
      sumDxDy += dx * dy;
      sumDxDz += dx * dz;
      sumDyDz += dy * dz;
    }

    double determinant = (sumDx2 * sumDy2) - (sumDxDy * sumDxDy);
    double slopeX = 0.0;
    double slopeY = 0.0;
    if (Math.abs(determinant) > 1e-9) {
      slopeX = ((sumDxDz * sumDy2) - (sumDyDz * sumDxDy)) / determinant;
      slopeY = ((sumDyDz * sumDx2) - (sumDxDz * sumDxDy)) / determinant;
    }

    double intercept = meanZ - (slopeX * meanX) - (slopeY * meanY);
    return new TerrainPlane(intercept, slopeX, slopeY, normalizeVector(-slopeX, -slopeY, 1.0));
  }

  private double sampleTerrainHeightAt(TerrainPlane plane, double xMeters, double yMeters) {
    return plane.interceptMeters()
        + (plane.slopeXMetersPerMeter() * xMeters)
        + (plane.slopeYMetersPerMeter() * yMeters);
  }

  private Rotation3d buildTerrainAlignedRotation(Rotation2d heading, Translation3d surfaceNormal) {
    double headingX = heading.getCos();
    double headingY = heading.getSin();
    double headingDotNormal = (headingX * surfaceNormal.getX()) + (headingY * surfaceNormal.getY());
    Translation3d projectedForward =
        new Translation3d(
            headingX - (headingDotNormal * surfaceNormal.getX()),
            headingY - (headingDotNormal * surfaceNormal.getY()),
            -headingDotNormal * surfaceNormal.getZ());

    Translation3d forward =
        vectorMagnitude(projectedForward) > 1e-9
            ? normalizeVector(
                projectedForward.getX(), projectedForward.getY(), projectedForward.getZ())
            : new Translation3d(headingX, headingY, 0.0);
    Translation3d left =
        normalizeVector(
            surfaceNormal.getY() * forward.getZ() - surfaceNormal.getZ() * forward.getY(),
            surfaceNormal.getZ() * forward.getX() - surfaceNormal.getX() * forward.getZ(),
            surfaceNormal.getX() * forward.getY() - surfaceNormal.getY() * forward.getX());
    Translation3d orthonormalForward =
        normalizeVector(
            left.getY() * surfaceNormal.getZ() - left.getZ() * surfaceNormal.getY(),
            left.getZ() * surfaceNormal.getX() - left.getX() * surfaceNormal.getZ(),
            left.getX() * surfaceNormal.getY() - left.getY() * surfaceNormal.getX());

    double m00 = orthonormalForward.getX();
    double m01 = left.getX();
    double m02 = surfaceNormal.getX();
    double m10 = orthonormalForward.getY();
    double m11 = left.getY();
    double m12 = surfaceNormal.getY();
    double m20 = orthonormalForward.getZ();
    double m21 = left.getZ();
    double m22 = surfaceNormal.getZ();

    double trace = m00 + m11 + m22;
    double qw;
    double qx;
    double qy;
    double qz;
    if (trace > 0.0) {
      double s = 2.0 * Math.sqrt(trace + 1.0);
      qw = 0.25 * s;
      qx = (m21 - m12) / s;
      qy = (m02 - m20) / s;
      qz = (m10 - m01) / s;
    } else if (m00 > m11 && m00 > m22) {
      double s = 2.0 * Math.sqrt(1.0 + m00 - m11 - m22);
      qw = (m21 - m12) / s;
      qx = 0.25 * s;
      qy = (m01 + m10) / s;
      qz = (m02 + m20) / s;
    } else if (m11 > m22) {
      double s = 2.0 * Math.sqrt(1.0 + m11 - m00 - m22);
      qw = (m02 - m20) / s;
      qx = (m01 + m10) / s;
      qy = 0.25 * s;
      qz = (m12 + m21) / s;
    } else {
      double s = 2.0 * Math.sqrt(1.0 + m22 - m00 - m11);
      qw = (m10 - m01) / s;
      qx = (m02 + m20) / s;
      qy = (m12 + m21) / s;
      qz = 0.25 * s;
    }

    return new Rotation3d(new Quaternion(qw, qx, qy, qz));
  }

  private Translation3d normalizeVector(double x, double y, double z) {
    double magnitude = Math.sqrt((x * x) + (y * y) + (z * z));
    if (magnitude <= 1e-9) {
      return new Translation3d(0.0, 0.0, 1.0);
    }
    return new Translation3d(x / magnitude, y / magnitude, z / magnitude);
  }

  private double vectorMagnitude(Translation3d vector) {
    return Math.sqrt(
        (vector.getX() * vector.getX())
            + (vector.getY() * vector.getY())
            + (vector.getZ() * vector.getZ()));
  }

  private double sampleGroundHeightAt(Translation2d position) {
    double blueHeight = sampleSingleHumpHeight(Constants.blueHub.getX(), position);
    double redHeight = sampleSingleHumpHeight(Constants.redHub.getX(), position);
    return Math.max(blueHeight, redHeight);
  }

  private double sampleSingleHumpHeight(double hubCenterXMeters, Translation2d position) {
    double maxHumpXDistance = (hubWidthXMeters / 2.0) + humpXClearanceMarginMeters;
    if (Math.abs(position.getX() - hubCenterXMeters) > maxHumpXDistance) {
      return 0.0;
    }

    double yOffsetFromHub = position.getY() - Constants.blueHub.getY();
    double absYOffset = Math.abs(yOffsetFromHub);
    double halfTopLength = hubTopLengthYMeters / 2.0;
    if (absYOffset > (halfTopLength + hubRampLengthYMeters)) {
      return 0.0;
    }

    if (absYOffset <= halfTopLength) {
      return humpPeakHeightMeters;
    }

    double rampTravelMeters = absYOffset - halfTopLength;
    double rampPercent = 1.0 - MathUtil.clamp(rampTravelMeters / hubRampLengthYMeters, 0.0, 1.0);
    return humpPeakHeightMeters * smoothStep01(rampPercent);
  }

  private double smoothStep01(double value) {
    double clampedValue = MathUtil.clamp(value, 0.0, 1.0);
    return clampedValue * clampedValue * (3.0 - (2.0 * clampedValue));
  }

  private record HumpPoseSample(
      double[] moduleHeightsMeters, double heightMeters, Translation3d surfaceNormal) {}

  private record TerrainPlane(
      double interceptMeters,
      double slopeXMetersPerMeter,
      double slopeYMetersPerMeter,
      Translation3d surfaceNormal) {}
}
