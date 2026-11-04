package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.Constants.CanBudget;
import frc.robot.Constants.CanIds;
import frc.robot.Constants.Gyro;

/** REAL HARDWARE: CTRE Pigeon 2.0 on the roboRIO CAN bus (no CANivore). */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon;
  private final StatusSignal<Angle> yaw;
  private final StatusSignal<AngularVelocity> yawRate;

  public GyroIOPigeon2(CANBus ctreBus) {
    pigeon = new Pigeon2(CanIds.PIGEON, ctreBus);

    Pigeon2Configuration cfg = new Pigeon2Configuration();
    cfg.MountPose.MountPoseYaw = Gyro.MOUNT_YAW_DEG;
    cfg.MountPose.MountPosePitch = Gyro.MOUNT_PITCH_DEG;
    cfg.MountPose.MountPoseRoll = Gyro.MOUNT_ROLL_DEG;
    StatusCode status = pigeon.getConfigurator().apply(cfg);
    if (!status.isOK()) {
      DriverStation.reportError("[Swerve] Pigeon config failed: " + status, false);
    }

    yaw = pigeon.getYaw();                       // degrees, CCW+
    yawRate = pigeon.getAngularVelocityZWorld(); // deg/s, for latency compensation
    // Opt in to ONLY yaw + yaw rate, then turn every other Pigeon signal off.
    BaseStatusSignal.setUpdateFrequencyForAll(CanBudget.PIGEON_YAW_HZ, yaw, yawRate);
    pigeon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(Inputs inputs) {
    // Non-blocking batched read of the latest frames.
    BaseStatusSignal.refreshAll(yaw, yawRate);
    inputs.connected = yaw.getStatus().isOK();
    // Latency compensation: uses yaw rate to predict yaw "right now".
    double yawDeg = BaseStatusSignal.getLatencyCompensatedValueAsDouble(yaw, yawRate);
    inputs.yaw = Rotation2d.fromDegrees(yawDeg);
    inputs.yawRateRadPerSec = Math.toRadians(yawRate.getValueAsDouble());
  }
}
