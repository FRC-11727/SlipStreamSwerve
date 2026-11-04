package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.Resync;
import frc.robot.Constants.Swerve;

/**
 * One swerve module's logic, shared by the real robot and the simulator.
 * The hardware details live in a {@link ModuleIO} ({@link ModuleIOSparkFlex}
 * on the robot, {@link ModuleIOSim} in simulation).
 *
 * <p>This class handles:
 * <ul>
 *   <li>Seeding the steering encoder from the CANcoder at boot.</li>
 *   <li>Re-syncing it if the two ever disagree while the wheel is still
 *       (e.g. a SPARK rebooted during a brownout).</li>
 *   <li>Wheel-angle optimization, cosine scaling and anti-jitter.</li>
 *   <li>Dashboard data.</li>
 * </ul>
 */
public class SwerveModule {
  private final String name;
  private final ModuleIO io;
  private final ModuleIO.Inputs inputs = new ModuleIO.Inputs();

  private boolean seeded = false;
  private int mismatchLoops = 0;
  private int resyncCount = 0;
  private Rotation2d lastCommandedAngle = new Rotation2d();
  private SwerveModuleState lastSetpoint = new SwerveModuleState(0.0, new Rotation2d());

  public SwerveModule(String name, ModuleIO io) {
    this.name = name;
    this.io = io;

    io.updateInputs(inputs);
    seeded = trySeedFromAbsolute();
    if (!seeded) {
      DriverStation.reportError("[Swerve " + name + "] CANcoder not responding at boot; "
          + "this module will not drive until it reports.", false);
    }
  }

  /** Call once per loop, before any commands run. */
  public void periodic() {
    io.updateInputs(inputs);

    if (!seeded) {
      seeded = trySeedFromAbsolute();
      if (seeded) {
        DriverStation.reportWarning("[Swerve " + name + "] CANcoder found; steering enabled.", false);
      }
      return;
    }

    // Re-sync check: if the wheel is not turning and the internal encoder
    // disagrees with the CANcoder for a while, trust the CANcoder.
    if (!inputs.absOk) {
      mismatchLoops = 0;
      return;
    }
    double errorRad = MathUtil.angleModulus(inputs.absAngleRad - inputs.steerPositionRad);
    boolean wheelStill = Math.abs(inputs.steerVelocityRadPerSec) < Resync.MAX_STEER_VELOCITY_RAD_PER_S;

    if (wheelStill && Math.abs(errorRad) > Resync.MAX_ERROR_RAD) {
      mismatchLoops++;
    } else {
      mismatchLoops = 0;
    }
    if (mismatchLoops >= Resync.LOOPS_BEFORE_RESYNC) {
      io.resetSteerEncoder(inputs.absAngleRad);
      mismatchLoops = 0;
      resyncCount++;
      DriverStation.reportWarning("[Swerve " + name + "] steering encoder re-synced to CANcoder ("
          + String.format("%.1f", Math.toDegrees(errorRad)) + " deg off)", false);
    }
  }

  private boolean trySeedFromAbsolute() {
    if (!inputs.absOk) {
      return false;
    }
    if (!io.resetSteerEncoder(inputs.absAngleRad)) {
      return false;
    }
    inputs.steerPositionRad = inputs.absAngleRad;
    lastCommandedAngle = Rotation2d.fromRadians(inputs.absAngleRad);
    return true;
  }

  // ==========================================================================
  //  Control
  // ==========================================================================

  /**
   * Command this module.
   *
   * @param desired    target speed (m/s) and angle
   * @param forceAngle true = turn to the angle even at zero speed (used for X-lock)
   */
  public void setDesiredState(SwerveModuleState desired, boolean forceAngle) {
    if (!seeded) {
      // Unknown steering angle: don't drive this module at all.
      io.stopMotors();
      return;
    }

    Rotation2d current = getSteerAngle();

    // Never rotate more than 90 deg: reverse the wheel instead.
    desired.optimize(current);
    // Slow the wheel while it's still pointing the wrong way (less "skating").
    desired.cosineScale(current);

    // Anti-jitter: at (almost) zero speed keep the last angle instead of
    // snapping wheels back to 0 deg when the driver lets go of the stick.
    boolean tooSlowToSteer =
        Math.abs(desired.speedMetersPerSecond) < 0.01 * Swerve.MAX_SPEED_MPS;
    Rotation2d targetAngle = (tooSlowToSteer && !forceAngle) ? lastCommandedAngle : desired.angle;
    lastCommandedAngle = targetAngle;
    lastSetpoint = new SwerveModuleState(desired.speedMetersPerSecond, targetAngle);

    io.setDriveVelocity(desired.speedMetersPerSecond);
    io.setSteerAngle(targetAngle.getRadians());
  }

  /** Stop driving but keep holding the current wheel angle. */
  public void stop() {
    if (!seeded) {
      io.stopMotors();
      return;
    }
    lastSetpoint = new SwerveModuleState(0.0, lastCommandedAngle);
    io.setDriveVelocity(0.0);
    io.setSteerAngle(lastCommandedAngle.getRadians());
  }

  // ==========================================================================
  //  Sensors
  // ==========================================================================

  /** Steering angle from the fast internal encoder. */
  public Rotation2d getSteerAngle() {
    return Rotation2d.fromRadians(MathUtil.angleModulus(inputs.steerPositionRad));
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(inputs.driveVelocityMps, getSteerAngle());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(inputs.drivePositionM, getSteerAngle());
  }

  /** What this module was last told to do (for AdvantageScope). */
  public SwerveModuleState getSetpoint() {
    return lastSetpoint;
  }

  public void applyGains(double dP, double dI, double dD, double dS, double dV, double dA,
                         double sP, double sI, double sD) {
    io.applyGains(dP, dI, dD, dS, dV, dA, sP, sI, sD);
  }

  // ==========================================================================
  //  Dashboard
  // ==========================================================================

  public void publishTelemetry() {
    String p = "Swerve/" + name + "/";
    SmartDashboard.putNumber(p + "Abs Raw (rot)", inputs.absRawRotations);
    SmartDashboard.putNumber(p + "Steer Abs (deg)", Math.toDegrees(inputs.absAngleRad));
    SmartDashboard.putNumber(p + "Steer Rel (deg)", getSteerAngle().getDegrees());
    SmartDashboard.putNumber(p + "Steer Target (deg)", lastCommandedAngle.getDegrees());
    SmartDashboard.putNumber(p + "Speed (mps)", inputs.driveVelocityMps);
    SmartDashboard.putNumber(p + "Drive Current (A)", inputs.driveCurrentA);
    SmartDashboard.putBoolean(p + "CANcoder OK", inputs.absOk);
    SmartDashboard.putBoolean(p + "Seeded", seeded);
    SmartDashboard.putNumber(p + "Resync Count", resyncCount);
  }
}
