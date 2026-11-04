package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * ============================================================================
 *  TEAM 11727 SLIPSTREAM — SWERVE CONSTANTS
 * ============================================================================
 *  Hardware: 8x NEO Vortex on SPARK Flex, 4x CTRE CANcoder, 1x Pigeon 2.0,
 *            SDS MK4n modules, REV PDH, roboRIO 2.0. ONE CAN bus (no CANivore).
 *
 *  Everything you are expected to change is marked with:   // >>> EDIT
 *  Search the file for ">>> EDIT" and walk through each one.
 *
 *  Units: meters, seconds, radians unless a name says otherwise.
 *  WPILib coordinate convention: +X = robot front, +Y = robot left,
 *  +rotation = counter-clockwise when viewed from above.
 * ============================================================================
 */
public final class Constants {
  private Constants() {}

  // ==========================================================================
  //  CAN IDs
  // ==========================================================================
  //  IDs only have to be unique per device TYPE, but keeping every ID unique
  //  across the whole bus makes debugging much easier. Set Spark IDs in the
  //  REV Hardware Client and CTRE IDs in Phoenix Tuner X.
  //
  //  REV PDH: leave it at CAN ID 1 (the WPILib default for a REV PDH). It is
  //  not created in code because the drivetrain doesn't need it; it still
  //  broadcasts on the bus, and the CAN budget in the README accounts for it.
  // ==========================================================================
  public static final class CanIds {
    // >>> EDIT: match these to what you set in REV Hardware Client / Tuner X
    public static final int FL_DRIVE = 10, FL_STEER = 11, FL_CANCODER = 12;
    public static final int FR_DRIVE = 20, FR_STEER = 21, FR_CANCODER = 22;
    public static final int BL_DRIVE = 30, BL_STEER = 31, BL_CANCODER = 32;
    public static final int BR_DRIVE = 40, BR_STEER = 41, BR_CANCODER = 42;
    public static final int PIGEON = 50;
    // No CANivore: the code puts the CTRE devices on the roboRIO's own CAN bus
    // with CANBus.roboRIO() (see SwerveSubsystem).
  }

  // ==========================================================================
  //  CAN BUS OPTIMIZATION
  // ==========================================================================
  //  Everything shares one 1 Mbit/s CAN 2.0 bus, so every status frame costs
  //  bandwidth. These rates are the ONLY telemetry the drivetrain asks for.
  //  All PID runs ON the SPARK Flex itself at 1 kHz, so these rates do not
  //  limit control quality; they only limit how often the roboRIO sees data.
  //
  //  Rough load at these settings: ~15-20% bus utilization (see README).
  //  Rule of thumb: keep "CAN Utilization %" on the dashboard under ~60%.
  // ==========================================================================
  public static final class CanBudget {
    /** SPARK Flex encoder position + velocity frame. Needed for odometry. */
    public static final int SPARK_ENCODER_PERIOD_MS = 20;      // 50 Hz = robot loop rate
    /** SPARK applied output / bus voltage / current / temp (they share a frame). */
    public static final int SPARK_TELEMETRY_PERIOD_MS = 100;   // 10 Hz (default is 100 Hz!)
    /** SPARK faults/warnings frame. */
    public static final int SPARK_FAULTS_PERIOD_MS = 250;

    /** Pigeon yaw + yaw rate. Yaw rate is used for latency compensation. */
    public static final double PIGEON_YAW_HZ = 100.0;
    /**
     * CANcoder absolute position. The CANcoder is only used to seed and
     * re-check the steering motor's internal encoder, so it can be slow.
     */
    public static final double CANCODER_HZ = 10.0;
  }
  
  // ==========================================================================
  //  MK4n MODULE + DRIVETRAIN GEOMETRY
  // ==========================================================================
  public static final class Swerve {
    // ---- Gear ratios (SDS MK4n) -------------------------------------------
    // MK4n drive ratios:  L1+ = 7.13:1   L2+ = 5.90:1   L3+ = 5.36:1
    public static final double MK4N_L1_PLUS = (50.0 / 16.0) * (19.0 / 25.0) * (45.0 / 15.0); // 7.125
    public static final double MK4N_L2_PLUS = (50.0 / 16.0) * (17.0 / 27.0) * (45.0 / 15.0); // 5.903
    public static final double MK4N_L3_PLUS = (50.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0); // 5.357

    // >>> EDIT: pick the ratio your modules were actually built with
    public static final double DRIVE_GEAR_RATIO = MK4N_L2_PLUS;

    /** MK4n steering reduction is 18.75:1 for every gearing option. */
    public static final double STEER_GEAR_RATIO = 18.75;

    // >>> EDIT: measure your wheel. New SDS 4" wheels are ~4.0", worn tread
    // is closer to 3.9". Wrong diameter = odometry distance error.
    public static final double WHEEL_DIAMETER_M = Units.inchesToMeters(4.0);
    public static final double WHEEL_CIRCUMFERENCE_M = WHEEL_DIAMETER_M * Math.PI;

    // >>> EDIT: measure CENTER-OF-WHEEL to CENTER-OF-WHEEL, not frame size.
    /** Left-right distance between wheel centers. */
    public static final double TRACK_WIDTH_M = Units.inchesToMeters(18);
    /** Front-back distance between wheel centers. */
    public static final double WHEEL_BASE_M = Units.inchesToMeters(17);

    /** Module positions relative to robot center. Order everywhere: FL, FR, BL, BR. */
    public static final Translation2d[] MODULE_TRANSLATIONS = {
      new Translation2d(+WHEEL_BASE_M / 2.0, +TRACK_WIDTH_M / 2.0), // Front Left
      new Translation2d(+WHEEL_BASE_M / 2.0, -TRACK_WIDTH_M / 2.0), // Front Right
      new Translation2d(-WHEEL_BASE_M / 2.0, +TRACK_WIDTH_M / 2.0), // Back Left
      new Translation2d(-WHEEL_BASE_M / 2.0, -TRACK_WIDTH_M / 2.0)  // Back Right
    };

    // ---- Speed limits --------------------------------------------------------
    /** NEO Vortex free speed (REV spec). */
    public static final double VORTEX_FREE_SPEED_RPM = 6784.0;
    /** Theoretical wheel free speed with your ratio. */
    public static final double DRIVE_FREE_SPEED_MPS =
        (VORTEX_FREE_SPEED_RPM / 60.0) / DRIVE_GEAR_RATIO * WHEEL_CIRCUMFERENCE_M;

    // >>> EDIT (optional): real robots reach ~85-90% of free speed. Lower this
    // to make the robot easier to drive while drivers are learning.
    public static final double MAX_SPEED_MPS = DRIVE_FREE_SPEED_MPS * 0.9;

    public static final double DRIVE_BASE_RADIUS_M =
        Math.hypot(TRACK_WIDTH_M / 2.0, WHEEL_BASE_M / 2.0);
    //public static final double MAX_ANGULAR_SPEED_RAD_PER_S = MAX_SPEED_MPS / DRIVE_BASE_RADIUS_M;
    public static final double PHYSICAL_MAX_ANGULAR_SPEED_RAD_PER_S = 
                  MAX_SPEED_MPS / DRIVE_BASE_RADIUS_M;

    public static final double MAX_SPIN_DEG_PER_S = 360.0;
    public static final double MAX_ANGULAR_SPEED_RAD_PER_S = 
                  Math.toRadians(MAX_SPIN_DEG_PER_S);

    // ---- Motor directions ---------------------------------------------------
    // MK4n steering motors must be inverted so that positive output turns the
    // wheel counter-clockwise (viewed from above). Verify with the README test.
    // >>> EDIT only if the README "steering direction test" fails
    public static final boolean STEER_INVERTED = true;

    // ---- Current limits (amps) ------------------------------------------------
    // >>> EDIT (optional): lower DRIVE if you brown out; 60A is a common Vortex value
    public static final int DRIVE_CURRENT_LIMIT_A = 60;
    public static final int STEER_CURRENT_LIMIT_A = 30;
    public static final double NOMINAL_VOLTAGE = 12.0;
  }

  // ==========================================================================
  //  PER-MODULE SETTINGS
  // ==========================================================================
  /**
   * @param name           shows up on the dashboard
   * @param driveId        SPARK Flex CAN ID for the drive Vortex
   * @param steerId        SPARK Flex CAN ID for the steering Vortex
   * @param cancoderId     CANcoder CAN ID
   * @param offsetRotations CANcoder magnet offset in ROTATIONS (-0.5 to 0.5)
   * @param driveInverted  flip if this wheel drives backwards (see README)
   */
  public record ModuleConfig(
      String name, int driveId, int steerId, int cancoderId,
      double offsetRotations, boolean driveInverted) {}

  public static final class Modules {
    // >>> EDIT: CANcoder offsets. Calibration steps are in the README:
    //   1) Set all offsets to 0.0 and deploy.
    //   2) Point every wheel straight forward, bevel gears all facing the
    //      robot's LEFT side.
    //   3) Read "Swerve/<module>/Abs Raw (rot)" on the dashboard.
    //   4) Put the NEGATIVE of that number here and redeploy.
    // >>> EDIT: driveInverted — flip a module to true if it drives backwards.
    public static final ModuleConfig FRONT_LEFT = new ModuleConfig(
        "FL", CanIds.FL_DRIVE, CanIds.FL_STEER, CanIds.FL_CANCODER, 0.0, false);
    public static final ModuleConfig FRONT_RIGHT = new ModuleConfig(
        "FR", CanIds.FR_DRIVE, CanIds.FR_STEER, CanIds.FR_CANCODER, 0.0, false);
    public static final ModuleConfig BACK_LEFT = new ModuleConfig(
        "BL", CanIds.BL_DRIVE, CanIds.BL_STEER, CanIds.BL_CANCODER, 0.0, false);
    public static final ModuleConfig BACK_RIGHT = new ModuleConfig(
        "BR", CanIds.BR_DRIVE, CanIds.BR_STEER, CanIds.BR_CANCODER, 0.0, false);

    // >>> EDIT only if the README "CANcoder direction test" fails.
    // true  = counter-clockwise (viewed from the top / LED side) is positive.
    public static final boolean CANCODER_CCW_POSITIVE = true;
  }

  // ==========================================================================
  //  PID + FEEDFORWARD GAINS  (all of these run ON the SPARK Flex at 1 kHz)
  // ==========================================================================
  //  Units after conversion factors:
  //    Drive  -> setpoint in meters/second. kP output is duty cycle (-1..1)
  //              per (m/s) of error. kS, kV, kA are in VOLTS.
  //    Steer  -> setpoint in radians. kP output is duty cycle per radian.
  //
  //  Set TUNING_MODE = true to edit these live from the dashboard (Elastic /
  //  Shuffleboard / Glass) under "Tuning/...". Copy final values back here and
  //  set TUNING_MODE = false for competition.
  // ==========================================================================
  public static final class Gains {
    // >>> EDIT: set false at competition
    public static final boolean TUNING_MODE = true;

    // ---- Drive velocity loop ------------------------------------------------
    // >>> EDIT: tune. Start with feedforward (kS, kV), then add a little kP.
    public static final double DRIVE_KP = 0.08;
    public static final double DRIVE_KI = 0.0;
    public static final double DRIVE_KD = 0.0;
    /** Volts to overcome friction. Find it by raising voltage until the wheel just moves. */
    public static final double DRIVE_KS = 0.15;
    /** Volts per (m/s). Theory: 12 V / free speed. Real value is usually a bit higher. */
    public static final double DRIVE_KV = Swerve.NOMINAL_VOLTAGE / Swerve.DRIVE_FREE_SPEED_MPS;
    public static final double DRIVE_KA = 0.0;

    // ---- Steer position loop ------------------------------------------------
    // >>> EDIT: tune. Raise kP until the wheel snaps to angle with a tiny
    // overshoot, then back off ~20%. Add kD only if it oscillates.
    public static final double STEER_KP = 1.0;
    public static final double STEER_KI = 0.0;
    public static final double STEER_KD = 0.0;

    // ---- Optional heading hold (robot keeps its angle when you aren't rotating)
    // >>> EDIT: leave false until the gyro + steering are verified, then try it.
    public static final boolean HEADING_HOLD_ENABLED = false;
    public static final double HEADING_KP = 4.0; // rad/s of correction per rad of error
    public static final double HEADING_KD = 0.0;
  }

  // ==========================================================================
  //  STEERING ENCODER RE-SYNC (CANcoder -> SPARK Flex internal encoder)
  // ==========================================================================
  //  The steering PID uses the Vortex's built-in encoder (no CAN latency).
  //  The CANcoder seeds it at boot and is re-checked every loop. If the two
  //  disagree while the wheel is still (e.g. a SPARK rebooted from a brownout),
  //  the internal encoder is re-seeded automatically.
  // ==========================================================================
  public static final class Resync {
    public static final double MAX_ERROR_RAD = Units.degreesToRadians(3.0);
    public static final double MAX_STEER_VELOCITY_RAD_PER_S = 0.5;
    public static final int LOOPS_BEFORE_RESYNC = 10; // 10 loops = 0.2 s
  }

  // ==========================================================================
  //  PIGEON 2.0 MOUNTING
  // ==========================================================================
  public static final class Gyro {
    // >>> EDIT if the Pigeon is not mounted flat with its arrow (+X) pointing
    // to the robot's front. Values in degrees. See Pigeon 2 "Mount Pose" docs.
    public static final double MOUNT_YAW_DEG = 0.0;
    public static final double MOUNT_PITCH_DEG = 0.0;
    public static final double MOUNT_ROLL_DEG = 0.0;
  }

  // ==========================================================================
  //  DRIVER CONTROLS
  // ==========================================================================
  public static final class OI {
    public static final int DRIVER_PORT = 0;          // >>> EDIT if needed
    public static final double DEADBAND = 0.17;       // >>> EDIT for your controller
    public static final double SLOW_MODE_SCALE = 0.35;
    /** Max change in joystick command per second (smooths acceleration). */
    public static final double TRANSLATION_SLEW = 3.0;
    public static final double ROTATION_SLEW = 0.1;
  }

  /** Robot loop period (TimedRobot default). */
  public static final double LOOP_PERIOD_S = 0.02;
}
