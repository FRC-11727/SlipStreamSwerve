package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.Gains;
import frc.robot.Constants.ModuleConfig;
import frc.robot.Constants.Modules;
import frc.robot.Constants.Swerve;

/**
 * Swerve drivetrain: 4x MK4n modules + gyro. Module order everywhere is FL, FR, BL, BR.
 *
 * <p>On the real robot this uses SPARK Flex + CANcoder + Pigeon 2.0 on the
 * roboRIO CAN bus. In the simulator it uses physics models instead. You don't
 * have to switch anything: it checks {@code RobotBase.isReal()} at startup.
 */
public class SwerveSubsystem extends SubsystemBase {
  private final SwerveModule[] modules;
  private final GyroIO gyroIO;
  private final GyroIO.Inputs gyroInputs = new GyroIO.Inputs();

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(Swerve.MODULE_TRANSLATIONS);
  private final SwerveDrivePoseEstimator poseEstimator;
  private final Field2d field = new Field2d();

  private ChassisSpeeds lastCommandedSpeeds = new ChassisSpeeds();

  // ---- Data for AdvantageScope (drag these onto its Field / Swerve tabs) ----
  private final NetworkTableInstance nt = NetworkTableInstance.getDefault();
  private final StructPublisher<Pose2d> posePub =
      nt.getStructTopic("Swerve/Pose", Pose2d.struct).publish();
  private final StructPublisher<Rotation2d> rotationPub =
      nt.getStructTopic("Swerve/Rotation", Rotation2d.struct).publish();
  private final StructArrayPublisher<SwerveModuleState> statesPub =
      nt.getStructArrayTopic("Swerve/States", SwerveModuleState.struct).publish();
  private final StructArrayPublisher<SwerveModuleState> setpointsPub =
      nt.getStructArrayTopic("Swerve/Setpoints", SwerveModuleState.struct).publish();

  public SwerveSubsystem() {
    if (RobotBase.isReal()) {
      // REAL ROBOT. The CTRE devices are on the roboRIO's built-in CAN bus
      // (no CANivore), together with the SPARK Flexes and the PDH.
      CANBus ctreBus = CANBus.roboRIO();
      gyroIO = new GyroIOPigeon2(ctreBus);
      modules = new SwerveModule[] {
        realModule(Modules.FRONT_LEFT, ctreBus),
        realModule(Modules.FRONT_RIGHT, ctreBus),
        realModule(Modules.BACK_LEFT, ctreBus),
        realModule(Modules.BACK_RIGHT, ctreBus)
      };
    } else {
      // SIMULATOR on your laptop.
      gyroIO = new GyroIOSim();
      modules = new SwerveModule[] {
        new SwerveModule(Modules.FRONT_LEFT.name(), new ModuleIOSim()),
        new SwerveModule(Modules.FRONT_RIGHT.name(), new ModuleIOSim()),
        new SwerveModule(Modules.BACK_LEFT.name(), new ModuleIOSim()),
        new SwerveModule(Modules.BACK_RIGHT.name(), new ModuleIOSim())
      };
    }

    gyroIO.updateInputs(gyroInputs);
    // In the simulator, start in open space near the blue wall instead of the
    // field's corner, so the robot is easy to see.
    Pose2d startPose = RobotBase.isReal()
        ? new Pose2d()
        : new Pose2d(new Translation2d(3.0, 4.0), new Rotation2d());
    poseEstimator = new SwerveDrivePoseEstimator(
        kinematics, gyroInputs.yaw, getModulePositions(), startPose);

    SmartDashboard.putData("Field", field);
    if (Gains.TUNING_MODE) {
      initTuning();
    }
  }

  private static SwerveModule realModule(ModuleConfig config, CANBus ctreBus) {
    return new SwerveModule(config.name(), new ModuleIOSparkFlex(config, ctreBus));
  }

  @Override
  public void periodic() {
    for (SwerveModule m : modules) {
      m.periodic();
    }

    // In simulation, spin the fake gyro by however fast the wheels say the
    // robot is rotating. (On the real robot the Pigeon measures it.)
    if (gyroIO instanceof GyroIOSim simGyro) {
      simGyro.integrate(
          kinematics.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond,
          Constants.LOOP_PERIOD_S);
    }
    gyroIO.updateInputs(gyroInputs);

    poseEstimator.update(gyroInputs.yaw, getModulePositions());
    field.setRobotPose(getPose());

    if (Gains.TUNING_MODE) {
      updateTuning();
    }
    publishTelemetry();
  }

  // ==========================================================================
  //  Driving
  // ==========================================================================

  /**
   * Drive with robot-relative speeds.
   *
   * @param robotRelative vx forward (m/s), vy left (m/s), omega CCW (rad/s)
   */
  public void driveRobotRelative(ChassisSpeeds robotRelative) {
    // Discretize: compensates for the robot rotating during the 20 ms loop,
    // which otherwise makes the robot drift sideways while spinning + driving.
    ChassisSpeeds discrete = ChassisSpeeds.discretize(robotRelative, Constants.LOOP_PERIOD_S);
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(discrete);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Swerve.MAX_SPEED_MPS);
    setModuleStates(states, false);
    lastCommandedSpeeds = robotRelative;
  }

  /**
   * Drive with field-relative speeds (WPILib field: +X away from the BLUE
   * alliance wall). The teleop command flips joystick input on red.
   */
  public void driveFieldRelative(ChassisSpeeds fieldRelative) {
    driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelative, getHeading()));
  }

  public void setModuleStates(SwerveModuleState[] states, boolean forceAngle) {
    for (int i = 0; i < 4; i++) {
      modules[i].setDesiredState(states[i], forceAngle);
    }
  }

  /** Point wheels into an X so the robot is hard to push. */
  public void setX() {
    setModuleStates(new SwerveModuleState[] {
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)),   // FL
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)),  // FR
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)),  // BL
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(45))    // BR
    }, true);
    lastCommandedSpeeds = new ChassisSpeeds();
  }

  public void stop() {
    for (SwerveModule m : modules) {
      m.stop();
    }
    lastCommandedSpeeds = new ChassisSpeeds();
  }

  public Command xLockCommand() {
    return run(this::setX).withName("Swerve X-Lock");
  }

  // ==========================================================================
  //  Odometry / heading
  // ==========================================================================

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Field-relative robot heading (from the pose estimator, not raw gyro). */
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(gyroInputs.yaw, getModulePositions(), pose);
  }

  /**
   * "Robot is now facing straight away from MY driver station."
   * Blue: heading 0 deg. Red: heading 180 deg (WPILib's blue-origin field).
   * Does NOT write to the Pigeon (avoids a blocking CAN config call).
   */
  public void zeroHeading() {
    boolean red = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
        == DriverStation.Alliance.Red;
    Rotation2d facing = red ? Rotation2d.fromDegrees(180.0) : new Rotation2d();
    resetPose(new Pose2d(getPose().getTranslation(), facing));
  }

  public Command zeroHeadingCommand() {
    return runOnce(this::zeroHeading).ignoringDisable(true).withName("Zero Heading");
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  private SwerveModuleState[] getModuleSetpoints() {
    SwerveModuleState[] setpoints = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      setpoints[i] = modules[i].getSetpoint();
    }
    return setpoints;
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  public boolean isGyroConnected() {
    return gyroInputs.connected;
  }

  // ==========================================================================
  //  Telemetry
  // ==========================================================================

  private void publishTelemetry() {
    // AdvantageScope data
    posePub.set(getPose());
    rotationPub.set(getHeading());
    statesPub.set(getModuleStates());
    setpointsPub.set(getModuleSetpoints());

    // Dashboard numbers
    SmartDashboard.putNumber("Swerve/Heading (deg)", getHeading().getDegrees());
    SmartDashboard.putBoolean("Swerve/Gyro OK", isGyroConnected());
    SmartDashboard.putNumber("Swerve/Cmd vx", lastCommandedSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Swerve/Cmd vy", lastCommandedSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Swerve/Cmd omega", lastCommandedSpeeds.omegaRadiansPerSecond);
    // Watch this on the real robot: it should stay well under ~60%.
    SmartDashboard.putNumber("CAN Utilization %",
        RobotController.getCANStatus().percentBusUtilization * 100.0);
    for (SwerveModule m : modules) {
      m.publishTelemetry();
    }
  }

  // ==========================================================================
  //  Live tuning (Gains.TUNING_MODE) — works on the robot AND in simulation
  // ==========================================================================
  private static final String[] TUNE_KEYS = {
    "Tuning/Drive kP", "Tuning/Drive kI", "Tuning/Drive kD",
    "Tuning/Drive kS", "Tuning/Drive kV", "Tuning/Drive kA",
    "Tuning/Steer kP", "Tuning/Steer kI", "Tuning/Steer kD"
  };
  private final double[] tuneValues = {
    Gains.DRIVE_KP, Gains.DRIVE_KI, Gains.DRIVE_KD,
    Gains.DRIVE_KS, Gains.DRIVE_KV, Gains.DRIVE_KA,
    Gains.STEER_KP, Gains.STEER_KI, Gains.STEER_KD
  };

  private void initTuning() {
    for (int i = 0; i < TUNE_KEYS.length; i++) {
      SmartDashboard.putNumber(TUNE_KEYS[i], tuneValues[i]);
    }
  }

  private void updateTuning() {
    boolean changed = false;
    for (int i = 0; i < TUNE_KEYS.length; i++) {
      double v = SmartDashboard.getNumber(TUNE_KEYS[i], tuneValues[i]);
      if (v != tuneValues[i]) {
        tuneValues[i] = v;
        changed = true;
      }
    }
    if (changed) {
      // Only sends CAN parameter frames when a number was actually edited.
      for (SwerveModule m : modules) {
        m.applyGains(tuneValues[0], tuneValues[1], tuneValues[2],
            tuneValues[3], tuneValues[4], tuneValues[5],
            tuneValues[6], tuneValues[7], tuneValues[8]);
      }
      DriverStation.reportWarning("[Swerve] New gains applied (not saved — "
          + "copy them into Constants.java).", false);
    }
  }
}
