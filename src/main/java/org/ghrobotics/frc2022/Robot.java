// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2022;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.ghrobotics.frc2022.commands.ClimbAutomatic;
import org.ghrobotics.frc2022.commands.ClimbReset;
import org.ghrobotics.frc2022.commands.ClimbTeleop;
import org.ghrobotics.frc2022.commands.DriveTeleop;
import org.ghrobotics.frc2022.subsystems.Climber;
import org.ghrobotics.frc2022.subsystems.Drivetrain;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // Initialize robot state.
  RobotState robot_state_ = new RobotState();

  // Create subsystems.
  private final Drivetrain drivetrain_ = new Drivetrain(robot_state_);
  private final Climber climber_ = new Climber();

  // Create Xbox controller for driver.
  private final XboxController driver_controller_ = new XboxController(0);

  // Keeps track of whether we are in climb mode.
  private boolean climb_mode_ = false;

  @Override
  public void robotInit() {
    // Disable LiveWindow telemetry.
    LiveWindow.disableAllTelemetry();

    // Enable NetworkTables flush() at higher rate.
    setNetworkTablesFlushEnabled(true);

    // Reset robot state.
    robot_state_.resetPosition(new Pose2d());

    // Set default commands for subsystems.
    drivetrain_.setDefaultCommand(new DriveTeleop(drivetrain_, driver_controller_));
    climber_.setDefaultCommand(new ClimbTeleop(climber_, driver_controller_, () -> climb_mode_));

    // Setup teleop controls.
    setupTeleopControls();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void autonomousInit() {
    // Reset climber (if not in climb mode).
    if (!climb_mode_)
      new ClimbReset(climber_).schedule();
  }

  @Override
  public void teleopInit() {
    // Reset climber (if not in climb mode).
    if (!climb_mode_)
      new ClimbReset(climber_).schedule();

  }

  @Override
  public void testInit() {}

  @Override
  public void robotPeriodic() {
    // Run command scheduler.
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testPeriodic() {}

  /**
   * Configures button / joystick bindings for teleop control (non-climb mode) if they are not
   * already configured in the respective subsystem default commands.
   */
  private void setupTeleopControls() {
    // Go into climb mode with B button.
    new JoystickButton(driver_controller_, XboxController.Button.kB.value)
        .whenPressed(() -> {
          climb_mode_ = true;
          CommandScheduler.getInstance().clearButtons();
          setupEndgameControls();
        });
  }

  /**
   * Configures button / joystick bindings for endgame control (climb mode) if they are not
   * already configured in the respective subsystem default commands.
   */
  private void setupEndgameControls() {
    // Go out of climb mode with B button.
    new JoystickButton(driver_controller_, XboxController.Button.kB.value)
        .whenPressed(() -> {
          climb_mode_ = false;
          CommandScheduler.getInstance().clearButtons();
          setupTeleopControls();
        });

    // Toggle automatic climb with Start button.
    new JoystickButton(driver_controller_, XboxController.Button.kStart.value)
        .toggleWhenPressed(new ClimbAutomatic(climber_, driver_controller_::getAButton));
  }
}
