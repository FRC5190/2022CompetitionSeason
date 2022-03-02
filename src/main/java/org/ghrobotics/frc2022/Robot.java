// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2022;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.ghrobotics.frc2022.commands.ClimbAutomatic;
import org.ghrobotics.frc2022.commands.ClimbTeleop;
import org.ghrobotics.frc2022.commands.DriveTeleop;
import org.ghrobotics.frc2022.subsystems.Climber;
import org.ghrobotics.frc2022.subsystems.Drivetrain;
import org.ghrobotics.frc2022.subsystems.LED;

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
  private final LED led_ = new LED();

  // Create Xbox controller for driver.
  private final XboxController driver_controller_ = new XboxController(0);

  // Keeps track of whether we are in climb mode.
  private boolean climb_mode_ = false;

  // Keeps track of whether we need to clear buttons.
  private boolean clear_buttons_ = false;

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
//    if (!climb_mode_)
//      new ClimbReset(climber_).schedule();
    climber_.enableSoftLimits(false);
  }

  @Override
  public void teleopInit() {
    // Reset climber (if not in climb mode).
//    if (!climb_mode_)
//      new ClimbReset(climber_).schedule();
    climber_.enableSoftLimits(false);
  }

  @Override
  public void testInit() {}

  @Override
  public void robotPeriodic() {
    // Run command scheduler.
    CommandScheduler.getInstance().run();

    // Send climb status to SmartDashboard.
    SmartDashboard.putBoolean("Climb Mode", climb_mode_);

    // Check if we need to clear buttons.
    if (clear_buttons_) {
      CommandScheduler.getInstance().clearButtons();
      // Setup correct controls based on climb mode.
      if (climb_mode_)
        setupEndgameControls();
      else
        setupTeleopControls();

      // Set flag to false.
      clear_buttons_ = false;
    }

    // Update LEDs.
    updateLEDs();
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
          clear_buttons_ = true;
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
          clear_buttons_ = true;
        });

    // Toggle automatic climb with Start button.
    new JoystickButton(driver_controller_, XboxController.Button.kStart.value)
        .toggleWhenPressed(new ClimbAutomatic(climber_, driver_controller_::getAButton));
  }

  /**
   * Updates the status of the LEDs periodically based on the various states of the robot (e.g.
   * climb mode, scoring, ball in intake).
   */
  private void updateLEDs() {
    // Climb Mode
    if (climb_mode_)
      led_.setOutput(LED.StandardLEDOutput.CLIMBING);

    // No Limelight (TODO)

    // Robot Disabled
    else if (isDisabled())
      led_.setOutput(LED.OutputType.RAINBOW);

    // Manual Scoring (TODO)

    // Automatic Scoring (TODO)

    // Other Cases
    else
      led_.setOutput(LED.StandardLEDOutput.BLANK);
  }
}
