package frc.robot.subsystems.gamepiece.indexers;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;

public class IndexersIOSparkMax implements IndexersIO {
  private final SparkMax topIndexer =
      new SparkMax(IndexersConstants.topIndexerCanId, MotorType.kBrushless);
  private final SparkMax bottomIndexer =
      new SparkMax(IndexersConstants.bottomIndexerCanId, MotorType.kBrushless);
  private double topPositionRotations = 0.0;
  private double bottomPositionRotations = 0.0;

  public IndexersIOSparkMax() {
    SparkBaseConfig brakeConfig =
        new SparkMaxConfig()
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(IndexersConstants.indexerMotorCurrentLimitAmps)
            .voltageCompensation(12.0);
    topIndexer.configure(
        brakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    bottomIndexer.configure(
        brakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateInputs(IndexersIOInputs inputs) {
    double topAppliedOutput = topIndexer.get();
    double bottomAppliedOutput = bottomIndexer.get();
    double topVelocityRotationsPerSec =
        topAppliedOutput * IndexersConstants.estimatedIndexerMaxVelocityRotationsPerSec;
    double bottomVelocityRotationsPerSec =
        bottomAppliedOutput * IndexersConstants.estimatedIndexerMaxVelocityRotationsPerSec;

    topPositionRotations += topVelocityRotationsPerSec * 0.02;
    bottomPositionRotations += bottomVelocityRotationsPerSec * 0.02;

    inputs.topConnected = true;
    inputs.bottomConnected = true;
    inputs.topPositionRotations = topPositionRotations;
    inputs.topVelocityRotationsPerSec = topVelocityRotationsPerSec;
    inputs.topAppliedOutput = topAppliedOutput;
    inputs.topCurrentAmps = topIndexer.getOutputCurrent();
    inputs.bottomPositionRotations = bottomPositionRotations;
    inputs.bottomVelocityRotationsPerSec = bottomVelocityRotationsPerSec;
    inputs.bottomAppliedOutput = bottomAppliedOutput;
    inputs.bottomCurrentAmps = bottomIndexer.getOutputCurrent();
  }

  @Override
  public void setTopOutput(double output) {
    topIndexer.set(output);
  }

  @Override
  public void setTopVelocitySetpointRotationsPerSec(double velocityRotationsPerSec) {
    // These real indexer paths are currently sensorless, so approximate velocity requests with
    // bounded duty-cycle commands until feedback hardware is added.
    topIndexer.set(
        MathUtil.clamp(
            velocityRotationsPerSec / IndexersConstants.estimatedIndexerMaxVelocityRotationsPerSec,
            -1.0,
            1.0));
  }

  @Override
  public void setBottomOutput(double output) {
    bottomIndexer.set(output);
  }

  @Override
  public void setBottomVelocitySetpointRotationsPerSec(double velocityRotationsPerSec) {
    bottomIndexer.set(
        MathUtil.clamp(
            velocityRotationsPerSec / IndexersConstants.estimatedIndexerMaxVelocityRotationsPerSec,
            -1.0,
            1.0));
  }
}
