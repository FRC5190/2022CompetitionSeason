// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2022;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.ghrobotics.frc2022.auto.RTarmacFenderFiveBall;
import org.ghrobotics.frc2022.commands.ClimbAutomatic;
import org.ghrobotics.frc2022.commands.ClimbReset;
import org.ghrobotics.frc2022.commands.ClimbTeleop;
import org.ghrobotics.frc2022.commands.DriveTeleop;
import org.ghrobotics.frc2022.subsystems.Climber;
import org.ghrobotics.frc2022.subsystems.Drivetrain;
import org.ghrobotics.frc2022.subsystems.Feeder;
import org.ghrobotics.frc2022.subsystems.Hood;
import org.ghrobotics.frc2022.subsystems.Intake;
import org.ghrobotics.frc2022.subsystems.LED;
import org.ghrobotics.frc2022.subsystems.LimelightManager;
import org.ghrobotics.frc2022.subsystems.Shooter;
import org.ghrobotics.frc2022.subsystems.Turret;
import org.ghrobotics.frc2022.vision.GoalTracker;
import static com.revrobotics.CANSparkMax.IdleMode;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // Initialize robot state.
  RobotState robot_state_ = new RobotState();

  // Initialize goal tracker.
  GoalTracker goal_tracker_ = new GoalTracker();

  // Create subsystems.
  private final Drivetrain drivetrain_ = new Drivetrain(robot_state_);
  private final Turret turret_ = new Turret(robot_state_);
  private final Shooter shooter_ = new Shooter();
  private final Hood hood_ = new Hood(robot_state_);
  private final Intake intake_ = new Intake();
  private final Feeder feeder_ = new Feeder();
  private final Climber climber_ = new Climber();
  private final LED led_ = new LED();

  // Create Limelight Manager.
  private final LimelightManager limelight_manager_ =
      new LimelightManager(robot_state_, goal_tracker_);

  // Create superstructure.
  private final Superstructure superstructure_ = new Superstructure(
      turret_, shooter_, hood_, intake_, feeder_, goal_tracker_, robot_state_);

  // Create autonomous mode selector.
  private SendableChooser<Command> auto_selector_ = new SendableChooser<>();
  private Command autonomous_command_ = null;

  // Create Xbox controller for driver.
  private final XboxController driver_controller_ = new XboxController(0);

  // Keeps track of whether we are in climb mode / climb reset.
  private final Command climb_reset_cmd_ = new ClimbReset(climber_);
  private boolean climb_mode_ = false;

  // Keeps track of whether we need to clear buttons.
  private boolean clear_buttons_ = false;

  // Create telemetry.
  private final Telemetry telemetry_ = new Telemetry(
      robot_state_, drivetrain_, turret_, shooter_, hood_, intake_, feeder_, climber_,
      auto_selector_, () -> climb_mode_);

  @Override
  public void robotInit() {
    // Disable LiveWindow telemetry.
    LiveWindow.disableAllTelemetry();

    // Enable NetworkTables flush() at higher rate.
    setNetworkTablesFlushEnabled(true);

    // Reset robot state.
    robot_state_.resetPosition(new Pose2d());

    // Setup auto.
    setupAuto();

    // Set default commands for subsystems:
    setDefaultCommands();

    // Setup teleop controls.
    setupTeleopControls();
  }

  @Override
  public void disabledInit() {
    // Set coast mode on drivetrain and turret to make them easier to move.
    drivetrain_.setIdleMode(IdleMode.kCoast);
    turret_.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void autonomousInit() {
    // Set brake mode on turret and drivetrain.
    drivetrain_.setIdleMode(IdleMode.kBrake);
    turret_.setIdleMode(IdleMode.kBrake);

    // Start autonomous program.
    autonomous_command_ = auto_selector_.getSelected();
    if (autonomous_command_ != null)
      autonomous_command_.schedule();
  }

  @Override
  public void teleopInit() {
    // Temporarily disable soft limits.
    climber_.enableSoftLimits(false);

    // Set brake mode on turret and drivetrain.
    drivetrain_.setIdleMode(IdleMode.kBrake);
    turret_.setIdleMode(IdleMode.kBrake);

    // Cancel autonomous program.
    if (autonomous_command_ != null)
      autonomous_command_.cancel();
  }

  @Override
  public void robotPeriodic() {
    // Run command scheduler.
    CommandScheduler.getInstance().run();

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

  /**
   * Creates auto modes and adds them to the selector.
   */
  private void setupAuto() {
    auto_selector_.addOption("Right Tarmac Fender High Goal 5 Ball",
        new RTarmacFenderFiveBall(robot_state_, drivetrain_, superstructure_));
  }

  /**
   * Sets default commands for each subsystem.
   */
  private void setDefaultCommands() {
    // Drivetrain:
    drivetrain_.setDefaultCommand(new DriveTeleop(drivetrain_, driver_controller_));

    // Turret:
    turret_.setDefaultCommand(new RunCommand(() -> turret_.setPercent(0), turret_));
//    turret_.setDefaultCommand(new RunCommand(() -> turret_.setGoal(Math.PI, 0), turret_));

    // Shooter:
    shooter_.setDefaultCommand(new RunCommand(() -> shooter_.setPercent(0), shooter_));

    // Hood:
    hood_.setDefaultCommand(new RunCommand(() -> hood_.setPercent(0), hood_));
//    hood_.setDefaultCommand(superstructure_.trackGoalWithHood());

    // Climber:
    climber_.setDefaultCommand(new ClimbTeleop(climber_, driver_controller_, () -> climb_mode_));
  }

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
          new RunCommand(() -> turret_.setGoal(Math.toRadians(90), 0), turret_).schedule();
        });


    new JoystickButton(driver_controller_, XboxController.Button.kA.value)
        .whenHeld(new RunCommand(() -> hood_.setPosition(Math.toRadians(25)), hood_));

    new JoystickButton(driver_controller_, XboxController.Button.kY.value)
        .whenHeld(new RunCommand(() -> hood_.setPosition(Math.toRadians(2.6)), hood_));

    // Intake with Left Trigger.
    new Button(() -> driver_controller_.getLeftTriggerAxis() > 0.1)
        .whenHeld(superstructure_.intake());

    // Shoot low goal with Left Bumper.
    new JoystickButton(driver_controller_, XboxController.Button.kLeftBumper.value)
        .whenHeld(superstructure_.scoreLowGoal());

    // Shoot high goal with Right Bumper.
    new JoystickButton(driver_controller_, XboxController.Button.kRightBumper.value)
        .whenHeld(superstructure_.scoreHighGoal());

    // Add field-relative turret hints with d-pad.
    new Button(() -> driver_controller_.getPOV() == 0)
        .whenHeld(new RunCommand(() -> turret_.setGoal(0, 0), turret_));
    new Button(() -> driver_controller_.getPOV() == 90)
        .whenHeld(new RunCommand(() -> turret_.setGoal(Math.toRadians(90), 0), turret_));
    new Button(() -> driver_controller_.getPOV() == 180)
        .whenHeld(new RunCommand(() -> turret_.setGoal(Math.toRadians(180), 0), turret_));
    new Button(() -> driver_controller_.getPOV() == 270)
        .whenHeld(new RunCommand(() -> turret_.setGoal(Math.toRadians(270), 0), turret_));
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
          turret_.getDefaultCommand().schedule();
        });

    // Reset climber when Back is pressed.
    new JoystickButton(driver_controller_, XboxController.Button.kBack.value)
        .whenHeld(climb_reset_cmd_);

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
    if (climb_reset_cmd_.isScheduled())
      led_.setOutput(LED.StandardLEDOutput.CLIMB_RESETTING);

    else if (climb_mode_)
      led_.setOutput(LED.StandardLEDOutput.CLIMBING);

      // No Limelight
    else if (!limelight_manager_.isLimelightAlive())
      led_.setOutput(LED.StandardLEDOutput.NO_LIMELIGHT);

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
