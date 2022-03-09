// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2022;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.ghrobotics.frc2022.auto.AutoPlanner;
import org.ghrobotics.frc2022.auto.RTFenderHG5Ball;
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
  // Constants
  public static final boolean kUsePoseEstimator = true;
  public static final boolean kUsePoseEstimatorInAuto = false;

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

  // Create superstructure and associated commands.
  private final Superstructure superstructure_ = new Superstructure(
      turret_, shooter_, hood_, intake_, feeder_, goal_tracker_, robot_state_);
  private final Command score_low_goal_fender_ = superstructure_.scoreLowGoalFender();
  private final Command score_high_goal_fender_ = superstructure_.scoreHighGoalFender();
  private final Command score_high_goal_ = superstructure_.scoreHighGoal();
  private final Command tune_shooter_ = superstructure_.tuneScoring();

  // Create autonomous mode selector.
  private final SendableChooser<Command> auto_selector_ = new SendableChooser<>();
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
      superstructure_,
      auto_selector_, () -> climb_mode_);

  @Override
  public void robotInit() {
    // Disable LiveWindow telemetry.
    LiveWindow.disableAllTelemetry();

    // Silence joystick warnings in sim.
    if (RobotBase.isSimulation())
      DriverStation.silenceJoystickConnectionWarning(true);

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

    robot_state_.resetPosition(AutoPlanner.kRTarmacFenderWallToBottomCargo.getInitialPose());
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
    robot_state_.resetPosition(AutoPlanner.kRTarmacFenderWallToBottomCargo.getInitialPose());

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

    // Update superstructure.
    superstructure_.periodic();

    // Update telemetry.
    telemetry_.periodic();

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
        new RTFenderHG5Ball(robot_state_, drivetrain_, superstructure_));
  }

  /**
   * Sets default commands for each subsystem.
   */
  private void setDefaultCommands() {
    // Drivetrain:
    drivetrain_.setDefaultCommand(new DriveTeleop(drivetrain_, driver_controller_));

    // Turret:
    turret_.setDefaultCommand(superstructure_.trackGoalWithTurret());

    // Shooter:
    shooter_.setDefaultCommand(new RunCommand(() -> shooter_.setPercent(0), shooter_));

    // Hood:
    hood_.setDefaultCommand(superstructure_.trackGoalWithHood());

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

    // TESTING: turret
    new JoystickButton(driver_controller_, XboxController.Button.kA.value)
        .whenHeld(new RunCommand(() -> turret_.setGoal(Math.toRadians(270), 0), turret_));

    // TESTING: turret
    new JoystickButton(driver_controller_, XboxController.Button.kY.value)
        .whenHeld(new RunCommand(() -> turret_.setGoal(Math.toRadians(90), 0), turret_));

    // Intake with Left Trigger.
    new Button(() -> driver_controller_.getLeftTriggerAxis() > 0.1)
        .whenHeld(superstructure_.intake());

    // Shoot low goal from fender with Left Bumper.
    new JoystickButton(driver_controller_, XboxController.Button.kLeftBumper.value)
        .whenHeld(score_low_goal_fender_);

    // Shoot high goal from fender with Right Bumper.
    new JoystickButton(driver_controller_, XboxController.Button.kRightBumper.value)
        .whenHeld(score_high_goal_fender_);

    // Shoot high goal with Right Trigger.
    new Button(() -> driver_controller_.getRightTriggerAxis() > 0.1)
        .whenHeld(score_high_goal_);

    // Add field-relative turret hints with d-pad.
    new Button(() -> driver_controller_.getPOV() == 0)
        .whenHeld(new RunCommand(
            () -> turret_.setGoal(0 - robot_state_.getRobotPose().getRotation().getRadians(), 0),
            turret_));
    new Button(() -> driver_controller_.getPOV() == 90)
        .whenHeld(new RunCommand(() -> turret_.setGoal(
            Math.toRadians(90) - robot_state_.getRobotPose().getRotation().getRadians(), 0),
            turret_));
    new Button(() -> driver_controller_.getPOV() == 180)
        .whenHeld(new RunCommand(() -> turret_.setGoal(
            Math.toRadians(180) - robot_state_.getRobotPose().getRotation().getRadians(), 0),
            turret_));
    new Button(() -> driver_controller_.getPOV() == 270)
        .whenHeld(new RunCommand(() -> turret_.setGoal(
            Math.toRadians(270) - robot_state_.getRobotPose().getRotation().getRadians(), 0),
            turret_));
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
    if (climb_reset_cmd_.isScheduled())
      led_.setOutput(LED.StandardLEDOutput.CLIMB_RESETTING);

    else if (climb_mode_)
      led_.setOutput(LED.StandardLEDOutput.CLIMBING);

    else if (!limelight_manager_.isLimelightAlive())
      led_.setOutput(LED.StandardLEDOutput.NO_LIMELIGHT);

    else if (isDisabled())
      led_.setOutput(LED.OutputType.RAINBOW);

    else if (tune_shooter_.isScheduled() || score_low_goal_fender_.isScheduled() ||
        score_high_goal_fender_.isScheduled())
      led_.setOutput(LED.StandardLEDOutput.MANUAL_SCORING);

    else if (score_high_goal_.isScheduled())
      led_.setOutput(LED.StandardLEDOutput.AUTOMATIC_SCORING);

    else
      led_.setOutput(LED.StandardLEDOutput.BLANK);
  }
}
