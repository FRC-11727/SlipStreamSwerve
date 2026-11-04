package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * SIMULATION ONLY: a "gyro" that turns by however fast the simulated wheels
 * say the robot is spinning.
 */
public class GyroIOSim implements GyroIO {
  private double yawRad = 0.0;
  private double yawRateRadPerSec = 0.0;

  /** Called by the drivetrain each loop with the robot's spin rate from the wheels. */
  public void integrate(double omegaRadPerSec, double dtSeconds) {
    yawRateRadPerSec = omegaRadPerSec;
    yawRad += omegaRadPerSec * dtSeconds;
  }

  @Override
  public void updateInputs(Inputs inputs) {
    inputs.connected = true;
    inputs.yaw = Rotation2d.fromRadians(yawRad);
    inputs.yawRateRadPerSec = yawRateRadPerSec;
  }
}
