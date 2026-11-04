package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/** Do not add anything here. Start in Robot.java / RobotContainer.java. */
public final class Main {
  private Main() {}

  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
