// Copyright (c) 2021-2026 Littleton Robotics
// Copyright (c) 2026 Team 4687 Spartobots
//
// SPDX-License-Identifier: BSD-3-Clause

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.Queue;

/** IO implementation for NavX. */
public class GyroIONavX implements GyroIO {
  private final AHRS navX = new AHRS(NavXComType.kUSB1, (byte) odometryFrequency);
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  public GyroIONavX() {
    // navX.setAngleAdjustment(134);
    yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(navX::getAngle);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    double yawSign = navxYawInverted ? -1.0 : 1.0;

    inputs.connected = navX.isConnected();
    inputs.yawPosition = Rotation2d.fromDegrees(yawSign * navX.getAngle());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawSign * navX.getRawGyroZ());

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(yawSign * value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }

  @Override
  public void zeroYaw() {
    navX.zeroYaw();
  }

  @Override
  public void setAngle(Rotation2d angle) {
    navX.zeroYaw();
    navX.setAngleAdjustment(angle.getDegrees());
  }
}
