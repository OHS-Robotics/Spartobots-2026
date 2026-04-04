package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.moduleAngleHoldMinSpeedMetersPerSec;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.junit.jupiter.api.Test;

class ModuleTest {
  @Test
  void lowSpeedSetpointKeepsPreviousAngleAndStopsDriveMotor() {
    TestModuleIO io = new TestModuleIO();
    Module module = new Module(io, 0);

    io.turnPosition = Rotation2d.fromDegrees(15.0);
    module.periodic();

    SwerveModuleState firstState = new SwerveModuleState(2.0, Rotation2d.fromDegrees(75.0));
    module.runSetpoint(firstState);

    Rotation2d heldAngle = io.lastTurnSetpoint;
    io.turnPosition = heldAngle;
    module.periodic();

    SwerveModuleState idleState =
        new SwerveModuleState(
            moduleAngleHoldMinSpeedMetersPerSec * 0.5, Rotation2d.fromDegrees(-105.0));
    module.runSetpoint(idleState);

    assertEquals(0.0, idleState.speedMetersPerSecond, 1e-9);
    assertEquals(heldAngle.getRadians(), idleState.angle.getRadians(), 1e-9);
    assertEquals(heldAngle.getRadians(), io.lastTurnSetpoint.getRadians(), 1e-9);
    assertEquals(0.0, io.lastDriveVelocityRadPerSec, 1e-9);
    assertEquals(1, io.turnControllerResetCalls);

    module.runSetpoint(
        new SwerveModuleState(
            moduleAngleHoldMinSpeedMetersPerSec * 0.25, Rotation2d.fromDegrees(120.0)));
    assertEquals(1, io.turnControllerResetCalls);
  }

  @Test
  void explicitTurnOnlySetpointCanStillReorientModule() {
    TestModuleIO io = new TestModuleIO();
    Module module = new Module(io, 0);

    io.turnPosition = Rotation2d.kZero;
    module.periodic();

    SwerveModuleState turnOnlyState = new SwerveModuleState(0.0, Rotation2d.fromDegrees(90.0));
    module.runSetpoint(turnOnlyState, false);

    assertEquals(Math.PI / 2.0, io.lastTurnSetpoint.getRadians(), 1e-9);
    assertEquals(0.0, io.lastDriveVelocityRadPerSec, 1e-9);
  }

  @Test
  void steeringLagOptimizesDirectionAndReducesDriveSpeedUntilAligned() {
    TestModuleIO io = new TestModuleIO();
    Module module = new Module(io, 0);

    io.turnPosition = Rotation2d.kZero;
    module.periodic();
    module.runSetpoint(new SwerveModuleState(2.0, Rotation2d.kZero));

    io.turnPosition = Rotation2d.fromDegrees(160.0);
    module.periodic();

    SwerveModuleState nextState = new SwerveModuleState(2.0, Rotation2d.fromDegrees(10.0));
    module.runSetpoint(nextState);

    double expectedSpeedMetersPerSecond = -2.0 * Math.cos(Math.toRadians(30.0));
    assertEquals(
        expectedSpeedMetersPerSecond / DriveConstants.wheelRadiusMeters,
        io.lastDriveVelocityRadPerSec,
        1e-9);
    assertEquals(
        Rotation2d.fromDegrees(-170.0).getRadians(), io.lastTurnSetpoint.getRadians(), 1e-9);
    assertEquals(expectedSpeedMetersPerSecond, nextState.speedMetersPerSecond, 1e-9);
    assertEquals(Rotation2d.fromDegrees(-170.0).getRadians(), nextState.angle.getRadians(), 1e-9);
  }

  private static class TestModuleIO implements ModuleIO {
    private Rotation2d turnPosition = Rotation2d.kZero;
    private Rotation2d lastTurnSetpoint = Rotation2d.kZero;
    private double lastDriveVelocityRadPerSec = Double.NaN;
    private int turnControllerResetCalls = 0;

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
      inputs.driveConnected = true;
      inputs.turnConnected = true;
      inputs.turnPosition = turnPosition;
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
      lastDriveVelocityRadPerSec = velocityRadPerSec;
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
      lastTurnSetpoint = rotation;
    }

    @Override
    public void resetTurnPositionController() {
      turnControllerResetCalls++;
    }
  }
}
