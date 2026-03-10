package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.sim.SpartobotsArena2026Rebuilt;
import frc.robot.testing.WpilibTestSupport;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class DriveSimulationTest {
  private static final double EPSILON = 1e-6;
  private static final double PERIOD_SECONDS = 0.02;

  @BeforeEach
  void setUp() {
    WpilibTestSupport.resetSchedulerAndTime();
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.notifyNewData();
    SimulatedArena.overrideInstance(new SpartobotsArena2026Rebuilt(false));
  }

  @Test
  void setPoseResetsEstimatorAndSimulationWorldPose() {
    DriveHarness harness = createHarness();
    Pose2d targetPose = new Pose2d(1.4, 0.7, Rotation2d.fromDegrees(32.0));

    harness.drive.setPose(targetPose);

    assertEquals(targetPose.getX(), harness.drive.getPose().getX(), EPSILON);
    assertEquals(targetPose.getY(), harness.drive.getPose().getY(), EPSILON);
    assertEquals(
        targetPose.getRotation().getRadians(), harness.drive.getRotation().getRadians(), EPSILON);
    assertEquals(
        targetPose.getX(), harness.driveSimulation.getSimulatedDriveTrainPose().getX(), EPSILON);
    assertEquals(
        targetPose.getY(), harness.driveSimulation.getSimulatedDriveTrainPose().getY(), EPSILON);
    assertTrue(harness.drive.isPoseTrusted());
  }

  @Test
  void runVelocityMovesRobotForwardInSimulation() {
    DriveHarness harness = createHarness();

    for (int i = 0; i < 100; i++) {
      harness.drive.runVelocity(new ChassisSpeeds(1.0, 0.0, 0.0));
      SimulatedArena.getInstance().simulationPeriodic();
      WpilibTestSupport.stepScheduler(PERIOD_SECONDS);
    }

    Pose2d simulatedPose = harness.driveSimulation.getSimulatedDriveTrainPose();
    Pose2d estimatedPose = harness.drive.getPose();

    assertTrue(
        simulatedPose.getX() > 0.5, "MapleSim pose should move forward under commanded velocity");
    assertTrue(estimatedPose.getX() > 0.25, "Drive odometry should advance in simulation");
    assertTrue(harness.drive.isPoseTrusted());
  }

  private static DriveHarness createHarness() {
    SwerveDriveSimulation driveSimulation =
        new SwerveDriveSimulation(DriveConstants.mapleSimConfig, Pose2d.kZero);
    SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
    Drive drive =
        new Drive(
            new GyroIOSim(driveSimulation.getGyroSimulation()),
            new ModuleIOSim(driveSimulation.getModules()[0]),
            new ModuleIOSim(driveSimulation.getModules()[1]),
            new ModuleIOSim(driveSimulation.getModules()[2]),
            new ModuleIOSim(driveSimulation.getModules()[3]),
            driveSimulation::setSimulationWorldPose);
    return new DriveHarness(driveSimulation, drive);
  }

  private record DriveHarness(SwerveDriveSimulation driveSimulation, Drive drive) {}
}
