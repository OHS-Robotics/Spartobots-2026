package com.team4687.frc2026.subsystems.vision.fusion;

public record VisionFusionConfig(
    double maxPoseAmbiguity,
    double maxTranslationJumpMeters,
    double maxHeadingErrorRadians,
    double stdDevScaleSingleTag) {}

