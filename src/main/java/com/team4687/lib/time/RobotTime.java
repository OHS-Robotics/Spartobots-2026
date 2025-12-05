package com.team4687.lib.time;

import edu.wpi.first.wpilibj.Timer;

public class RobotTime {
    public static double getTimestampSeconds() {
        return Timer.getFPGATimestamp();
    }
}
