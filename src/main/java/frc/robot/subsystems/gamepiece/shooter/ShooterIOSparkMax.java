package frc.robot.subsystems.gamepiece.shooter;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

public class ShooterIOSparkMax implements ShooterIO {
  private final SparkFlex drumMotor40 =
      new SparkFlex(ShooterConstants.shooterLeaderCanId, MotorType.kBrushless);
  private final SparkFlex drumMotor41 =
      new SparkFlex(ShooterConstants.shooterFollowerOneCanId, MotorType.kBrushless);
  private final SparkMax drumMotor42 =
      new SparkMax(ShooterConstants.shooterFollowerTwoCanId, MotorType.kBrushless);
  private final SparkMax drumMotor43 =
      new SparkMax(ShooterConstants.shooterFollowerThreeCanId, MotorType.kBrushless);
  private final SparkMax hoodMotor = new SparkMax(ShooterConstants.hoodCanId, MotorType.kBrushless);

  private final RelativeEncoder drumMotor40Encoder = drumMotor40.getEncoder();
  private final RelativeEncoder hoodEncoder = hoodMotor.getEncoder();

  private final SparkClosedLoopController drumMotor40Controller =
      drumMotor40.getClosedLoopController();
  private final SparkClosedLoopController hoodController = hoodMotor.getClosedLoopController();

  private final Debouncer drumConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer hoodConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private double wheelVelocityKp = ShooterConstants.shooterVelocityKp;
  private double wheelVelocityKi = ShooterConstants.shooterVelocityKi;
  private double wheelVelocityKd = ShooterConstants.shooterVelocityKd;
  private double wheelVelocityKv = ShooterConstants.shooterVelocityKv;
  private double hoodPositionKp = ShooterConstants.hoodPositionKp;
  private double hoodPositionKi = ShooterConstants.hoodPositionKi;
  private double hoodPositionKd = ShooterConstants.hoodPositionKd;

  public ShooterIOSparkMax() {
    final double shooterVelocityConversionFactorRadPerSec = (2.0 * Math.PI) / 60.0;

    var drumMotor40Config = new SparkFlexConfig();
    drumMotor40Config
        .inverted(ShooterConstants.shooterMotor40Inverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ShooterConstants.shooterMotorCurrentLimitAmps)
        .voltageCompensation(12.0);
    drumMotor40Config
        .encoder
        .velocityConversionFactor(shooterVelocityConversionFactorRadPerSec)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    drumMotor40Config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(
            ShooterConstants.shooterVelocityKp,
            ShooterConstants.shooterVelocityKi,
            ShooterConstants.shooterVelocityKd);
    drumMotor40Config.closedLoop.feedForward.kV(ShooterConstants.shooterVelocityKv);
    drumMotor40Config
        .signals
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    var drumMotor41Config = new SparkFlexConfig();
    drumMotor41Config
        .follow(
            drumMotor40,
            ShooterConstants.shooterMotor41Inverted != ShooterConstants.shooterMotor40Inverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ShooterConstants.shooterMotorCurrentLimitAmps)
        .voltageCompensation(12.0);

    var drumMotor42Config = new SparkMaxConfig();
    drumMotor42Config
        .follow(
            drumMotor40,
            ShooterConstants.shooterMotor42Inverted != ShooterConstants.shooterMotor40Inverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ShooterConstants.shooterMotorCurrentLimitAmps)
        .voltageCompensation(12.0);

    var drumMotor43Config = new SparkMaxConfig();
    drumMotor43Config
        .follow(
            drumMotor40,
            ShooterConstants.shooterMotor43Inverted != ShooterConstants.shooterMotor40Inverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ShooterConstants.shooterMotorCurrentLimitAmps)
        .voltageCompensation(12.0);

    var hoodConfig = new SparkMaxConfig();
    hoodConfig
        .inverted(ShooterConstants.hoodInverted)
        .idleMode(IdleMode.kCoast)
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
        drumMotor40,
        5,
        () ->
            drumMotor40.configure(
                drumMotor40Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(
        drumMotor41,
        5,
        () ->
            drumMotor41.configure(
                drumMotor41Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(
        drumMotor42,
        5,
        () ->
            drumMotor42.configure(
                drumMotor42Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(
        drumMotor43,
        5,
        () ->
            drumMotor43.configure(
                drumMotor43Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
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
        drumMotor40,
        drumMotor40Encoder::getVelocity,
        (value) -> {
          inputs.pair1LeaderVelocityRadPerSec = value;
          inputs.pair2LeaderVelocityRadPerSec = value;
        });
    ifOk(
        drumMotor40,
        new DoubleSupplier[] {drumMotor40::getAppliedOutput, drumMotor40::getBusVoltage},
        (values) -> {
          inputs.pair1AppliedVolts = values[0] * values[1];
          inputs.pair2AppliedVolts = values[0] * values[1];
        });
    ifOk(
        drumMotor40,
        drumMotor40::getOutputCurrent,
        (value) -> {
          inputs.pair1CurrentAmps = value;
          inputs.pair2CurrentAmps = value;
        });
    boolean drumConnected =
        drumConnectedDebounce.calculate(
            !sparkStickyFault
                && isMotorResponsive(drumMotor40)
                && isMotorResponsive(drumMotor41)
                && isMotorResponsive(drumMotor42)
                && isMotorResponsive(drumMotor43));
    inputs.pair1Connected = drumConnected;
    inputs.pair2Connected = drumConnected;

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
    double sharedSetpoint = resolveSharedWheelSetpoint(pair1RadPerSec, pair2RadPerSec);
    drumMotor40Controller.setSetpoint(sharedSetpoint, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setHoodPositionSetpointRotations(double hoodPositionRotations) {
    hoodController.setSetpoint(hoodPositionRotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setHoodOpenLoopOutput(double output) {
    hoodMotor.set(MathUtil.clamp(output, -1.0, 1.0));
  }

  @Override
  public void setHoodEncoderPositionRotations(double hoodPositionRotations) {
    hoodEncoder.setPosition(hoodPositionRotations);
  }

  @Override
  public void setWheelVelocityClosedLoopGains(double kp, double ki, double kd, double kv) {
    if (!hasGainChange(wheelVelocityKp, kp)
        && !hasGainChange(wheelVelocityKi, ki)
        && !hasGainChange(wheelVelocityKd, kd)
        && !hasGainChange(wheelVelocityKv, kv)) {
      return;
    }

    wheelVelocityKp = kp;
    wheelVelocityKi = ki;
    wheelVelocityKd = kd;
    wheelVelocityKv = kv;

    var wheelConfig = new SparkFlexConfig();
    wheelConfig.closedLoop.pid(kp, ki, kd);
    wheelConfig.closedLoop.feedForward.kV(kv);

    tryUntilOk(
        drumMotor40,
        5,
        () ->
            drumMotor40.configure(
                wheelConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
  }

  @Override
  public void setHoodPositionClosedLoopGains(double kp, double ki, double kd) {
    if (!hasGainChange(hoodPositionKp, kp)
        && !hasGainChange(hoodPositionKi, ki)
        && !hasGainChange(hoodPositionKd, kd)) {
      return;
    }

    hoodPositionKp = kp;
    hoodPositionKi = ki;
    hoodPositionKd = kd;

    var hoodPidConfig = new SparkMaxConfig();
    hoodPidConfig.closedLoop.pid(kp, ki, kd);
    tryUntilOk(
        hoodMotor,
        5,
        () ->
            hoodMotor.configure(
                hoodPidConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
  }

  private boolean hasGainChange(double oldValue, double newValue) {
    return Math.abs(oldValue - newValue) > 1e-9;
  }

  private static double resolveSharedWheelSetpoint(double pair1RadPerSec, double pair2RadPerSec) {
    if (Math.abs(pair1RadPerSec) <= 1e-9) {
      return pair2RadPerSec;
    }
    if (Math.abs(pair2RadPerSec) <= 1e-9) {
      return pair1RadPerSec;
    }
    return 0.5 * (pair1RadPerSec + pair2RadPerSec);
  }

  @Override
  public void stop() {
    drumMotor40.setVoltage(0.0);
    hoodMotor.setVoltage(0.0);
  }

  private static boolean isMotorResponsive(SparkBase motor) {
    sparkStickyFault = false;
    ifOk(motor, motor::getAppliedOutput, value -> {});
    return !sparkStickyFault;
  }
}
