package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import frc.robot.Constants;
import frc.robot.Constants.Gains;
import frc.robot.Constants.Swerve;

/**
 * SIMULATION ONLY: a physics model of one MK4n module with two NEO Vortex
 * motors. Used automatically when you run "WPILib: Simulate Robot Code".
 *
 * <p>It copies what the SPARK Flex does on the real robot: the same PID gains
 * and feedforward from {@code Constants.Gains}, run 1000 times per second,
 * driving a simulated motor. So if the gains are wildly wrong, you'll see it
 * here too. (Real-world friction, backlash and battery sag are not modeled,
 * so final tuning still has to happen on the real robot.)
 */
public class ModuleIOSim implements ModuleIO {
  // >>> EDIT (optional): rough inertias the motors have to spin.
  // Drive includes a share of the robot's mass; steer is just the module.
  private static final double DRIVE_INERTIA_KG_M2 = 0.025;
  private static final double STEER_INERTIA_KG_M2 = 0.004;

  /** Run the "SPARK" control loop at 1 kHz like the real controller does. */
  private static final int SUBSTEPS = 20;
  private static final double SUB_DT = Constants.LOOP_PERIOD_S / SUBSTEPS;

  private final DCMotorSim driveSim;
  private final DCMotorSim steerSim;
  private final double wheelRadiusM = Swerve.WHEEL_DIAMETER_M / 2.0;

  // Setpoints (NaN = nothing commanded = motor off)
  private double driveSetpointMps = Double.NaN;
  private double steerSetpointRad = Double.NaN;

  // Gains (mirroring the SPARK Flex)
  private double dP = Gains.DRIVE_KP, dI = Gains.DRIVE_KI, dD = Gains.DRIVE_KD;
  private double dS = Gains.DRIVE_KS, dV = Gains.DRIVE_KV;
  private double sP = Gains.STEER_KP, sI = Gains.STEER_KI, sD = Gains.STEER_KD;

  private double driveIntegral = 0.0, lastDriveError = 0.0;
  private double steerIntegral = 0.0, lastSteerError = 0.0;

  /** The simulated steer encoder can be "re-seeded" like the real one. */
  private double steerEncoderOffsetRad = 0.0;

  public ModuleIOSim() {
    DCMotor vortex = DCMotor.getNeoVortex(1);
    driveSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(vortex, DRIVE_INERTIA_KG_M2, Swerve.DRIVE_GEAR_RATIO),
        vortex);
    steerSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(vortex, STEER_INERTIA_KG_M2, Swerve.STEER_GEAR_RATIO),
        vortex);
  }

  @Override
  public void updateInputs(Inputs inputs) {
    stepPhysics();

    double wheelAngleRad = steerSim.getAngularPositionRad();

    inputs.drivePositionM = driveSim.getAngularPositionRad() * wheelRadiusM;
    inputs.driveVelocityMps = driveSim.getAngularVelocityRadPerSec() * wheelRadiusM;
    inputs.driveCurrentA = Math.abs(driveSim.getCurrentDrawAmps());
    inputs.steerPositionRad = wheelAngleRad + steerEncoderOffsetRad;
    inputs.steerVelocityRadPerSec = steerSim.getAngularVelocityRadPerSec();
    // The simulated "CANcoder" always reads the true wheel angle.
    inputs.absAngleRad = MathUtil.angleModulus(wheelAngleRad);
    inputs.absRawRotations = inputs.absAngleRad / (2.0 * Math.PI);
    inputs.absOk = true;
  }

  /** Advance the physics by one robot loop, running the PID 20 times inside it. */
  private void stepPhysics() {
    boolean enabled = DriverStation.isEnabled();
    for (int i = 0; i < SUBSTEPS; i++) {
      double driveVolts = 0.0;
      double steerVolts = 0.0;

      if (enabled && !Double.isNaN(driveSetpointMps)) {
        double velocity = driveSim.getAngularVelocityRadPerSec() * wheelRadiusM;
        double error = driveSetpointMps - velocity;
        driveIntegral += error * SUB_DT;
        double derivative = (error - lastDriveError) / SUB_DT;
        lastDriveError = error;
        // SPARK PID output is duty cycle (-1..1); x12 V turns it into volts.
        double pid = (dP * error + dI * driveIntegral + dD * derivative) * Swerve.NOMINAL_VOLTAGE;
        double ff = dS * Math.signum(driveSetpointMps) + dV * driveSetpointMps;
        driveVolts = pid + ff;
      }

      if (enabled && !Double.isNaN(steerSetpointRad)) {
        double position = steerSim.getAngularPositionRad() + steerEncoderOffsetRad;
        // Same "position wrapping" the SPARK does: always take the short way.
        double error = MathUtil.angleModulus(steerSetpointRad - position);
        steerIntegral += error * SUB_DT;
        double derivative = (error - lastSteerError) / SUB_DT;
        lastSteerError = error;
        steerVolts = (sP * error + sI * steerIntegral + sD * derivative) * Swerve.NOMINAL_VOLTAGE;
      }

      driveSim.setInputVoltage(MathUtil.clamp(driveVolts, -Swerve.NOMINAL_VOLTAGE, Swerve.NOMINAL_VOLTAGE));
      steerSim.setInputVoltage(MathUtil.clamp(steerVolts, -Swerve.NOMINAL_VOLTAGE, Swerve.NOMINAL_VOLTAGE));
      driveSim.update(SUB_DT);
      steerSim.update(SUB_DT);
    }
  }

  @Override
  public void setDriveVelocity(double metersPerSecond) {
    driveSetpointMps = metersPerSecond;
  }

  @Override
  public void setSteerAngle(double radians) {
    steerSetpointRad = radians;
  }

  @Override
  public void stopMotors() {
    driveSetpointMps = Double.NaN;
    steerSetpointRad = Double.NaN;
    driveIntegral = 0.0;
    steerIntegral = 0.0;
  }

  @Override
  public boolean resetSteerEncoder(double radians) {
    steerEncoderOffsetRad = radians - steerSim.getAngularPositionRad();
    return true;
  }

  @Override
  public void applyGains(double driveP, double driveI, double driveD,
                         double driveS, double driveV, double driveA,
                         double steerP, double steerI, double steerD) {
    dP = driveP; dI = driveI; dD = driveD; dS = driveS; dV = driveV;
    sP = steerP; sI = steerI; sD = steerD;
  }
}
