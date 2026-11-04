package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.Gains;
import frc.robot.Constants.OI;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/**
 * Default teleop drive: left stick translates, right stick X rotates.
 * Field-relative by default; hold the robot-relative button to drive like a tank.
 */
public class TeleopDrive extends Command {
  private final SwerveSubsystem swerve;
  private final DoubleSupplier forward;   // + = away from driver
  private final DoubleSupplier strafe;    // + = to driver's left
  private final DoubleSupplier rotate;    // + = counter-clockwise
  private final BooleanSupplier robotRelative;
  private final BooleanSupplier slowMode;

  private final SlewRateLimiter xLimiter = new SlewRateLimiter(OI.TRANSLATION_SLEW);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(OI.TRANSLATION_SLEW);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(OI.ROTATION_SLEW);

  // Heading hold (optional, see Gains.HEADING_HOLD_ENABLED)
  private final PIDController headingPid = new PIDController(Gains.HEADING_KP, 0.0, Gains.HEADING_KD);
  private Rotation2d heldHeading = null;

  public TeleopDrive(SwerveSubsystem swerve, DoubleSupplier forward, DoubleSupplier strafe,
                     DoubleSupplier rotate, BooleanSupplier robotRelative, BooleanSupplier slowMode) {
    this.swerve = swerve;
    this.forward = forward;
    this.strafe = strafe;
    this.rotate = rotate;
    this.robotRelative = robotRelative;
    this.slowMode = slowMode;
    headingPid.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    heldHeading = null;
    xLimiter.reset(0.0);
    yLimiter.reset(0.0);
    rotLimiter.reset(0.0);
  }

  @Override
  public void execute() {
    double x = shape(forward.getAsDouble());
    double y = shape(strafe.getAsDouble());
    double r = shape(rotate.getAsDouble());

    double scale = slowMode.getAsBoolean() ? OI.SLOW_MODE_SCALE : 1.0;
    x = xLimiter.calculate(x * scale);
    y = yLimiter.calculate(y * scale);
    r = rotLimiter.calculate(r * scale);

    double vx = x * Swerve.MAX_SPEED_MPS;
    double vy = y * Swerve.MAX_SPEED_MPS;
    double omega = r * Swerve.MAX_ANGULAR_SPEED_RAD_PER_S;

    // ---- Optional heading hold: lock the angle when the driver isn't rotating
    if (Gains.HEADING_HOLD_ENABLED) {
      boolean rotating = Math.abs(r) > 0.0;
      boolean translating = Math.hypot(vx, vy) > 0.05;
      if (rotating || !translating) {
        heldHeading = null; // driver is turning (or robot is parked): don't fight it
      } else {
        if (heldHeading == null) {
          heldHeading = swerve.getHeading();
          headingPid.reset();
        }
        omega = headingPid.calculate(
            swerve.getHeading().getRadians(), heldHeading.getRadians());
      }
    }

    if (robotRelative.getAsBoolean()) {
      swerve.driveRobotRelative(new ChassisSpeeds(vx, vy, omega));
    } else {
      // WPILib field origin is the BLUE wall. On red, "away from me" is -X.
      boolean red = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
          == DriverStation.Alliance.Red;
      if (red) {
        vx = -vx;
        vy = -vy;
      }
      swerve.driveFieldRelative(new ChassisSpeeds(vx, vy, omega));
    }
  }

  /** Deadband, then square the input (keeping sign) for finer low-speed control. */
  private static double shape(double raw) {
    double v = MathUtil.applyDeadband(raw, OI.DEADBAND);
    return Math.copySign(v * v, v);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stop();
  }
}
