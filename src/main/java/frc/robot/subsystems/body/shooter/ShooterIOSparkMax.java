package frc.robot.subsystems.body.shooter;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

public class ShooterIOSparkMax implements ShooterIO {
  private final SparkBase pair1Leader =
      new SparkMax(ShooterConstants.pair1LeaderCanId, MotorType.kBrushless);
  private final SparkBase pair1Follower =
      new SparkMax(ShooterConstants.pair1FollowerCanId, MotorType.kBrushless);
  private final SparkBase pair2Leader =
      new SparkMax(ShooterConstants.pair2LeaderCanId, MotorType.kBrushless);
  private final SparkBase pair2Follower =
      new SparkMax(ShooterConstants.pair2FollowerCanId, MotorType.kBrushless);
  private final SparkBase hoodMotor =
      new SparkMax(ShooterConstants.hoodCanId, MotorType.kBrushless);

  private final RelativeEncoder pair1LeaderEncoder = pair1Leader.getEncoder();
  private final RelativeEncoder pair1FollowerEncoder = pair1Follower.getEncoder();
  private final RelativeEncoder pair2LeaderEncoder = pair2Leader.getEncoder();
  private final RelativeEncoder pair2FollowerEncoder = pair2Follower.getEncoder();
  private final RelativeEncoder hoodEncoder = hoodMotor.getEncoder();

  private final SparkClosedLoopController pair1Controller = pair1Leader.getClosedLoopController();
  private final SparkClosedLoopController pair2Controller = pair2Leader.getClosedLoopController();
  private final SparkClosedLoopController hoodController = hoodMotor.getClosedLoopController();

  private final Debouncer pair1ConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer pair2ConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer hoodConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public ShooterIOSparkMax() {
    final double shooterVelocityConversionFactorRadPerSec = (2.0 * Math.PI) / 60.0;

    var pair1LeaderConfig = new SparkMaxConfig();
    pair1LeaderConfig
        .inverted(ShooterConstants.pair1Inverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ShooterConstants.shooterMotorCurrentLimitAmps)
        .voltageCompensation(12.0);
    pair1LeaderConfig
        .encoder
        .velocityConversionFactor(shooterVelocityConversionFactorRadPerSec)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    pair1LeaderConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(
            ShooterConstants.shooterVelocityKp,
            ShooterConstants.shooterVelocityKi,
            ShooterConstants.shooterVelocityKd)
        .velocityFF(ShooterConstants.shooterVelocityKv);
    pair1LeaderConfig
        .signals
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    var pair1FollowerConfig = new SparkMaxConfig();
    pair1FollowerConfig
        .follow(pair1Leader, ShooterConstants.pairFollowerInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ShooterConstants.shooterMotorCurrentLimitAmps)
        .voltageCompensation(12.0);
    pair1FollowerConfig
        .encoder
        .velocityConversionFactor(shooterVelocityConversionFactorRadPerSec)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    pair1FollowerConfig
        .signals
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    var pair2LeaderConfig = new SparkMaxConfig();
    pair2LeaderConfig
        .inverted(ShooterConstants.pair2Inverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ShooterConstants.shooterMotorCurrentLimitAmps)
        .voltageCompensation(12.0);
    pair2LeaderConfig
        .encoder
        .velocityConversionFactor(shooterVelocityConversionFactorRadPerSec)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    pair2LeaderConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(
            ShooterConstants.shooterVelocityKp,
            ShooterConstants.shooterVelocityKi,
            ShooterConstants.shooterVelocityKd)
        .velocityFF(ShooterConstants.shooterVelocityKv);
    pair2LeaderConfig
        .signals
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    var pair2FollowerConfig = new SparkMaxConfig();
    pair2FollowerConfig
        .follow(pair2Leader, ShooterConstants.pairFollowerInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ShooterConstants.shooterMotorCurrentLimitAmps)
        .voltageCompensation(12.0);
    pair2FollowerConfig
        .encoder
        .velocityConversionFactor(shooterVelocityConversionFactorRadPerSec)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    pair2FollowerConfig
        .signals
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    var hoodConfig = new SparkMaxConfig();
    hoodConfig
        .inverted(ShooterConstants.hoodInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ShooterConstants.hoodMotorCurrentLimitAmps)
        .voltageCompensation(12.0);
    hoodConfig.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
    hoodConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(
            ShooterConstants.hoodPositionKp,
            ShooterConstants.hoodPositionKi,
            ShooterConstants.hoodPositionKd);
    hoodConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    tryUntilOk(
        pair1Leader,
        5,
        () ->
            pair1Leader.configure(
                pair1LeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(
        pair1Follower,
        5,
        () ->
            pair1Follower.configure(
                pair1FollowerConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
    tryUntilOk(
        pair2Leader,
        5,
        () ->
            pair2Leader.configure(
                pair2LeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(
        pair2Follower,
        5,
        () ->
            pair2Follower.configure(
                pair2FollowerConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
    tryUntilOk(
        hoodMotor,
        5,
        () ->
            hoodMotor.configure(
                hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(
        pair1Leader,
        pair1LeaderEncoder::getVelocity,
        (value) -> inputs.pair1LeaderVelocityRadPerSec = value);
    ifOk(
        pair1Follower,
        pair1FollowerEncoder::getVelocity,
        (value) -> inputs.pair1FollowerVelocityRadPerSec = value);
    ifOk(
        pair1Leader,
        new DoubleSupplier[] {pair1Leader::getAppliedOutput, pair1Leader::getBusVoltage},
        (values) -> inputs.pair1AppliedVolts = values[0] * values[1]);
    ifOk(pair1Leader, pair1Leader::getOutputCurrent, (value) -> inputs.pair1CurrentAmps = value);
    inputs.pair1Connected = pair1ConnectedDebounce.calculate(!sparkStickyFault);

    sparkStickyFault = false;
    ifOk(
        pair2Leader,
        pair2LeaderEncoder::getVelocity,
        (value) -> inputs.pair2LeaderVelocityRadPerSec = value);
    ifOk(
        pair2Follower,
        pair2FollowerEncoder::getVelocity,
        (value) -> inputs.pair2FollowerVelocityRadPerSec = value);
    ifOk(
        pair2Leader,
        new DoubleSupplier[] {pair2Leader::getAppliedOutput, pair2Leader::getBusVoltage},
        (values) -> inputs.pair2AppliedVolts = values[0] * values[1]);
    ifOk(pair2Leader, pair2Leader::getOutputCurrent, (value) -> inputs.pair2CurrentAmps = value);
    inputs.pair2Connected = pair2ConnectedDebounce.calculate(!sparkStickyFault);

    sparkStickyFault = false;
    ifOk(hoodMotor, hoodEncoder::getPosition, (value) -> inputs.hoodPositionRotations = value);
    ifOk(
        hoodMotor,
        hoodEncoder::getVelocity,
        (value) -> inputs.hoodVelocityRotationsPerSec = value / 60.0);
    ifOk(
        hoodMotor,
        new DoubleSupplier[] {hoodMotor::getAppliedOutput, hoodMotor::getBusVoltage},
        (values) -> inputs.hoodAppliedVolts = values[0] * values[1]);
    ifOk(hoodMotor, hoodMotor::getOutputCurrent, (value) -> inputs.hoodCurrentAmps = value);
    inputs.hoodConnected = hoodConnectedDebounce.calculate(!sparkStickyFault);
  }

  @Override
  public void setWheelVelocitySetpoints(double pair1RadPerSec, double pair2RadPerSec) {
    pair1Controller.setSetpoint(pair1RadPerSec, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    pair2Controller.setSetpoint(pair2RadPerSec, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setHoodPositionSetpointRotations(double hoodPositionRotations) {
    hoodController.setSetpoint(hoodPositionRotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void stop() {
    pair1Leader.setVoltage(0.0);
    pair2Leader.setVoltage(0.0);
    hoodMotor.setVoltage(0.0);
  }
}
