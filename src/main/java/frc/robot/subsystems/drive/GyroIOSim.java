// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.drive.DriveConstants.navxYawInverted;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.SparkUtil;
import java.util.Arrays;
import org.ironmaple.simulation.drivesims.GyroSimulation;

/** Physics sim implementation of gyro IO backed by MapleSim. */
public class GyroIOSim implements GyroIO {
  private final GyroSimulation gyroSimulation;

  public GyroIOSim(GyroSimulation gyroSimulation) {
    this.gyroSimulation = gyroSimulation;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    double yawSign = navxYawInverted ? -1.0 : 1.0;

    inputs.connected = true;
    inputs.yawPosition =
        Rotation2d.fromRadians(yawSign * gyroSimulation.getGyroReading().getRadians());
    inputs.yawVelocityRadPerSec =
        yawSign * gyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond);

    inputs.odometryYawTimestamps = SparkUtil.getSimulationOdometryTimeStamps();
    inputs.odometryYawPositions =
        Arrays.stream(gyroSimulation.getCachedGyroReadings())
            .map((yaw) -> Rotation2d.fromRadians(yawSign * yaw.getRadians()))
            .toArray(Rotation2d[]::new);
  }
}
