package frc.robot.subsystems.swerve;

/**
 * The "hardware layer" for one swerve module.
 *
 * <p>There are two versions of this:
 * <ul>
 *   <li>{@link ModuleIOSparkFlex} — the REAL robot (SPARK Flex + NEO Vortex + CANcoder).</li>
 *   <li>{@link ModuleIOSim} — a physics model used in the simulator on your laptop.</li>
 * </ul>
 * The robot picks the right one automatically. Everything above this layer
 * ({@link SwerveModule}, {@link SwerveSubsystem}, the drive command) is shared,
 * so what you test in simulation is the same logic that runs on the robot.
 */
public interface ModuleIO {

  /** Sensor readings, refreshed once per loop. */
  class Inputs {
    public double drivePositionM = 0.0;
    public double driveVelocityMps = 0.0;
    public double driveCurrentA = 0.0;

    /** Steering angle from the motor's internal encoder (can go past +/-PI). */
    public double steerPositionRad = 0.0;
    public double steerVelocityRadPerSec = 0.0;

    /** Absolute steering angle from the CANcoder, -PI..PI. */
    public double absAngleRad = 0.0;
    /** CANcoder reading in rotations — used for offset calibration. */
    public double absRawRotations = 0.0;
    /** False if the CANcoder isn't reporting. */
    public boolean absOk = false;
  }

  /** Read all sensors into {@code inputs}. */
  void updateInputs(Inputs inputs);

  /** Run the drive wheel at this speed (closed-loop velocity PID). */
  void setDriveVelocity(double metersPerSecond);

  /** Turn the wheel to this angle (closed-loop position PID). */
  void setSteerAngle(double radians);

  /** Cut power to both motors. */
  void stopMotors();

  /** Overwrite the steering motor's internal encoder. Returns true on success. */
  boolean resetSteerEncoder(double radians);

  /** Live gain tuning. */
  void applyGains(double driveP, double driveI, double driveD,
                  double driveS, double driveV, double driveA,
                  double steerP, double steerI, double steerD);
}
