// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2022;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.ghrobotics.frc2022.commands.DriveTeleop;
import org.ghrobotics.frc2022.subsystems.Drivetrain;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // Initialize robot state.
  RobotState robot_state = new RobotState();

  // Create subsystems.
  private final Drivetrain drivetrain_ = new Drivetrain(robot_state);
//  private final Climber climber_ = new Climber();

  // Create Xbox controller for driver.
  private final XboxController controller_ = new XboxController(0);

  // Keeps track of whether we are in climb mode.
  private boolean climb_mode_ = false;

  // The global climber command that is always used when climbing.
//  private final ClimbCommand climb_cmd_ = new ClimbCommand(climber_, controller_,
//      () -> climb_mode_);

  @Override
  public void robotInit() {
    // Disable LiveWindow telemetry.
    LiveWindow.disableAllTelemetry();

    // Enable NetworkTables flush() at higher rate.
    setNetworkTablesFlushEnabled(true);

    // Set default commands for subsystems.
    drivetrain_.setDefaultCommand(new DriveTeleop(drivetrain_, controller_));
//    climber_.setDefaultCommand(climb_cmd_);

    // Setup controls.
    setControls();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void teleopInit() {}

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
   * Configures the button / joystick bindings for teleoperated control.
   */
  private void setControls() {
    // Toggle climb mode with B button. Whenever this is pressed, the state of the climber is reset.
    new JoystickButton(controller_, XboxController.Button.kB.value)
        .whenPressed(() -> {
          climb_mode_ = !climb_mode_;
//          climb_cmd_.resetClimbState();
        });
  }
}
