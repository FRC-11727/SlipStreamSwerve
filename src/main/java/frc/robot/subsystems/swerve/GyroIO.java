package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The "hardware layer" for the gyro.
 * {@link GyroIOPigeon2} = real Pigeon 2.0, {@link GyroIOSim} = simulator.
 */
public interface GyroIO {
  class Inputs {
    public boolean connected = false;
    /** Counter-clockwise positive. */
    public Rotation2d yaw = new Rotation2d();
    public double yawRateRadPerSec = 0.0;
  }

  void updateInputs(Inputs inputs);
}
