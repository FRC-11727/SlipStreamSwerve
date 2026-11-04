package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.OI;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/**
 * Wires the drivetrain to the driver's Xbox controller.
 *
 * <p>Controls (>>> EDIT to taste):
 * <ul>
 *   <li>Left stick: drive (field-relative)</li>
 *   <li>Right stick X: rotate</li>
 *   <li>Start: zero heading (robot must be facing away from you)</li>
 *   <li>X (hold): X-lock wheels (defense)</li>
 *   <li>Left bumper (hold): robot-relative driving</li>
 *   <li>Right bumper (hold): slow mode</li>
 * </ul>
 */
public class RobotContainer {
  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final CommandXboxController driver = new CommandXboxController(OI.DRIVER_PORT);

  public RobotContainer() {
    // Xbox sticks: up = negative Y, right = positive X. WPILib wants
    // +X forward, +Y left, +rotation CCW, so every axis is negated.
    swerve.setDefaultCommand(new TeleopDrive(
        swerve,
        () -> -driver.getLeftY(),
        () -> -driver.getLeftX(),
        () -> -driver.getRightX(),
        driver.leftBumper(),
        driver.rightBumper()));

    configureBindings();
  }

  private void configureBindings() {
    driver.start().onTrue(swerve.zeroHeadingCommand());
    driver.x().whileTrue(swerve.xLockCommand());
  }

  /** Drivetrain-only code: no autonomous routine yet. */
  public Command getAutonomousCommand() {
    return Commands.none();
  }

  public SwerveSubsystem getSwerve() {
    return swerve;
  }
}
