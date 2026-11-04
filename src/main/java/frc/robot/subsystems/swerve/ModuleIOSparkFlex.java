package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.Constants.CanBudget;
import frc.robot.Constants.Gains;
import frc.robot.Constants.ModuleConfig;
import frc.robot.Constants.Modules;
import frc.robot.Constants.Swerve;

/**
 * REAL HARDWARE: one MK4n module with two NEO Vortex motors on SPARK Flex
 * controllers and a CTRE CANcoder, all on the roboRIO's CAN bus.
 *
 * <ul>
 *   <li>Drive: SPARK Flex onboard VELOCITY PID + feedforward (kS/kV/kA), m/s.</li>
 *   <li>Steer: SPARK Flex onboard POSITION PID on the Vortex's internal encoder,
 *       radians, with position wrapping so it always takes the short way around.</li>
 *   <li>CANcoder: only seeds/re-checks the internal steer encoder, so it can
 *       report slowly (10 Hz) and save bus bandwidth.</li>
 * </ul>
 * Both PID loops run on the SPARK Flex at 1 kHz, so they don't depend on CAN speed.
 */
public class ModuleIOSparkFlex implements ModuleIO {
  private final String name;

  private final SparkFlex driveMotor;
  private final SparkFlex steerMotor;
  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder steerEncoder;
  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController steerController;

  private final CANcoder cancoder;
  private final StatusSignal<Angle> absolutePosition;

  // Conversion factors: motor rotations -> meters / radians
  private static final double DRIVE_POSITION_FACTOR =
      Swerve.WHEEL_CIRCUMFERENCE_M / Swerve.DRIVE_GEAR_RATIO;            // m per motor rotation
  private static final double DRIVE_VELOCITY_FACTOR = DRIVE_POSITION_FACTOR / 60.0; // RPM -> m/s
  private static final double STEER_POSITION_FACTOR = 2.0 * Math.PI / Swerve.STEER_GEAR_RATIO; // rad
  private static final double STEER_VELOCITY_FACTOR = STEER_POSITION_FACTOR / 60.0; // RPM -> rad/s

  public ModuleIOSparkFlex(ModuleConfig config, CANBus ctreBus) {
    name = config.name();

    // ---------------- CANcoder (CTRE, on the roboRIO bus) ----------------
    cancoder = new CANcoder(config.cancoderId(), ctreBus);
    CANcoderConfiguration ccCfg = new CANcoderConfiguration();
    ccCfg.MagnetSensor.MagnetOffset = config.offsetRotations();
    ccCfg.MagnetSensor.SensorDirection = Modules.CANCODER_CCW_POSITIVE
        ? SensorDirectionValue.CounterClockwise_Positive
        : SensorDirectionValue.Clockwise_Positive;
    ccCfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5; // report -0.5..0.5 rotations
    checkCtre(cancoder.getConfigurator().apply(ccCfg), "CANcoder config");

    absolutePosition = cancoder.getAbsolutePosition();
    // Only ask for the ONE signal we use, then switch every other CANcoder
    // signal off. This is the main CTRE-side bus optimization.
    absolutePosition.setUpdateFrequency(CanBudget.CANCODER_HZ);
    cancoder.optimizeBusUtilization();

    // ---------------- Drive SPARK Flex ----------------
    driveMotor = new SparkFlex(config.driveId(), MotorType.kBrushless);
    SparkFlexConfig driveCfg = new SparkFlexConfig();
    driveCfg
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Swerve.DRIVE_CURRENT_LIMIT_A)
        .voltageCompensation(Swerve.NOMINAL_VOLTAGE)
        .inverted(config.driveInverted());
    driveCfg.encoder
        .positionConversionFactor(DRIVE_POSITION_FACTOR)
        .velocityConversionFactor(DRIVE_VELOCITY_FACTOR)
        // Less velocity filtering = less lag in the velocity PID.
        .quadratureMeasurementPeriod(10)
        .quadratureAverageDepth(2);
    driveCfg.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(Gains.DRIVE_KP, Gains.DRIVE_KI, Gains.DRIVE_KD)
        .outputRange(-1.0, 1.0);
    driveCfg.closedLoop.feedForward
        .kS(Gains.DRIVE_KS)
        .kV(Gains.DRIVE_KV)
        .kA(Gains.DRIVE_KA);
    applySignalRates(driveCfg);
    checkRev(driveMotor.configure(driveCfg, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters), "drive config");

    // ---------------- Steer SPARK Flex ----------------
    steerMotor = new SparkFlex(config.steerId(), MotorType.kBrushless);
    SparkFlexConfig steerCfg = new SparkFlexConfig();
    steerCfg
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Swerve.STEER_CURRENT_LIMIT_A)
        .voltageCompensation(Swerve.NOMINAL_VOLTAGE)
        .inverted(Swerve.STEER_INVERTED);
    steerCfg.encoder
        .positionConversionFactor(STEER_POSITION_FACTOR)
        .velocityConversionFactor(STEER_VELOCITY_FACTOR);
    steerCfg.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(Gains.STEER_KP, Gains.STEER_KI, Gains.STEER_KD)
        .outputRange(-1.0, 1.0)
        // Wrapping: -PI and +PI are the same angle, so the PID never spins
        // the long way around.
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(-Math.PI, Math.PI);
    applySignalRates(steerCfg);
    checkRev(steerMotor.configure(steerCfg, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters), "steer config");

    driveEncoder = driveMotor.getEncoder();
    steerEncoder = steerMotor.getEncoder();
    driveController = driveMotor.getClosedLoopController();
    steerController = steerMotor.getClosedLoopController();

    driveEncoder.setPosition(0.0);

    // Wait (up to 0.5 s) for one fresh CANcoder reading so the steering
    // encoder can be seeded at boot. Waiting on ONE signal is allowed on the
    // roboRIO bus.
    absolutePosition.waitForUpdate(0.5);
  }

  /**
   * SPARK status frame periods. REVLib 2025+ only turns on the frames for
   * signals you request, so NEVER call getAbsoluteEncoder()/getAnalog()/
   * getExternalEncoder() on these motors — that would turn extra frames on.
   */
  private static void applySignalRates(SparkFlexConfig cfg) {
    cfg.signals
        .primaryEncoderPositionPeriodMs(CanBudget.SPARK_ENCODER_PERIOD_MS)
        .primaryEncoderVelocityPeriodMs(CanBudget.SPARK_ENCODER_PERIOD_MS)
        // These four share one status frame; the smallest period wins.
        .appliedOutputPeriodMs(CanBudget.SPARK_TELEMETRY_PERIOD_MS)
        .busVoltagePeriodMs(CanBudget.SPARK_TELEMETRY_PERIOD_MS)
        .outputCurrentPeriodMs(CanBudget.SPARK_TELEMETRY_PERIOD_MS)
        .motorTemperaturePeriodMs(CanBudget.SPARK_TELEMETRY_PERIOD_MS)
        .faultsPeriodMs(CanBudget.SPARK_FAULTS_PERIOD_MS)
        .warningsPeriodMs(CanBudget.SPARK_FAULTS_PERIOD_MS);
  }

  @Override
  public void updateInputs(Inputs inputs) {
    // Non-blocking: grabs the latest CANcoder frame the roboRIO already has.
    absolutePosition.refresh();

    inputs.drivePositionM = driveEncoder.getPosition();
    inputs.driveVelocityMps = driveEncoder.getVelocity();
    inputs.driveCurrentA = driveMotor.getOutputCurrent();
    inputs.steerPositionRad = steerEncoder.getPosition();
    inputs.steerVelocityRadPerSec = steerEncoder.getVelocity();
    inputs.absOk = absolutePosition.getStatus().isOK();
    inputs.absRawRotations = absolutePosition.getValueAsDouble();
    inputs.absAngleRad = MathUtil.angleModulus(inputs.absRawRotations * 2.0 * Math.PI);
  }

  @Override
  public void setDriveVelocity(double metersPerSecond) {
    driveController.setSetpoint(metersPerSecond, SparkBase.ControlType.kVelocity);
  }

  @Override
  public void setSteerAngle(double radians) {
    steerController.setSetpoint(radians, SparkBase.ControlType.kPosition);
  }

  @Override
  public void stopMotors() {
    driveMotor.stopMotor();
    steerMotor.stopMotor();
  }

  @Override
  public boolean resetSteerEncoder(double radians) {
    return steerEncoder.setPosition(radians) == REVLibError.kOk;
  }

  /**
   * Push new gains without resetting other settings and without writing
   * flash. Only called when a value actually changed (it sends CAN parameter
   * frames).
   */
  @Override
  public void applyGains(double dP, double dI, double dD, double dS, double dV, double dA,
                         double sP, double sI, double sD) {
    SparkFlexConfig d = new SparkFlexConfig();
    d.closedLoop.pid(dP, dI, dD);
    d.closedLoop.feedForward.kS(dS).kV(dV).kA(dA);
    driveMotor.configureAsync(d, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    SparkFlexConfig s = new SparkFlexConfig();
    s.closedLoop.pid(sP, sI, sD);
    steerMotor.configureAsync(s, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  private void checkRev(REVLibError err, String what) {
    if (err != REVLibError.kOk) {
      DriverStation.reportError("[Swerve " + name + "] " + what + " failed: " + err, false);
    }
  }

  private void checkCtre(StatusCode code, String what) {
    if (!code.isOK()) {
      DriverStation.reportError("[Swerve " + name + "] " + what + " failed: " + code, false);
    }
  }
}
