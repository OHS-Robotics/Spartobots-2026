package com.team4687.frc2026.subsystems.body;

import edu.wpi.first.math.util.Units;

public final class ShooterConstants {
  public static final double minAngle = Units.degreesToRadians(47.0);   // the shooter can go up to a max of 45 degrees
  public static final double maxAngle = Units.degreesToRadians(90.0);   // the zero on the shooter is 90 degrees
  public static final double minPower = 0.02; // 2% power
  public static final double maxPower = 12.0; // 100% power

  public static final double preferredAngle = Units.degreesToRadians(75.0); // we want to prefer steeper angles so the balls don't hit the side of the hub
  public static final double preferredPower = maxPower * 0.8;  // 80% seems okay for now

  // todo: account for shooter height?
}