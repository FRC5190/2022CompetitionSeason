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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.ghrobotics.frc2022.auto.HighLeft2;
import org.ghrobotics.frc2022.auto.HighLeft2Steal2;
import org.ghrobotics.frc2022.auto.HighLeft4;
import org.ghrobotics.frc2022.auto.HighRight3;
import org.ghrobotics.frc2022.auto.HighRight3Steal1;
import org.ghrobotics.frc2022.auto.HighRight5;
import org.ghrobotics.frc2022.commands.ClimbAutomatic;
import org.ghrobotics.frc2022.commands.ClimbReset;
import org.ghrobotics.frc2022.commands.ClimbTeleop;
import org.ghrobotics.frc2022.commands.DriveTeleop;
import org.ghrobotics.frc2022.commands.TurretZero;
import org.ghrobotics.frc2022.subsystems.CargoTracker;
import org.ghrobotics.frc2022.subsystems.Climber;
import org.ghrobotics.frc2022.subsystems.Drivetrain;
import org.ghrobotics.frc2022.subsystems.Hood;
import org.ghrobotics.frc2022.subsystems.Intake;
import org.ghrobotics.frc2022.subsystems.LED;
import org.ghrobotics.frc2022.subsystems.LimelightManager;
import org.ghrobotics.frc2022.subsystems.Shooter;
import org.ghrobotics.frc2022.subsystems.Turret;

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

  // Create Xbox controller for driver.
  private final XboxController driver_controller_ = new XboxController(0);

  // Initialize robot state.
  private final RobotState robot_state_ = new RobotState();

  // Create subsystems.
  private final Drivetrain drivetrain_ = new Drivetrain(robot_state_);
  private final Turret turret_ = new Turret(robot_state_);
  private final Shooter shooter_ = new Shooter();
  private final Hood hood_ = new Hood(robot_state_);
  private final Intake intake_ = new Intake();
  private final Climber climber_ = new Climber();
  private final LED led_ = new LED();
  private final CargoTracker cargo_tracker_ = new CargoTracker(intake_, shooter_);
  private final LimelightManager limelight_manager_ = new LimelightManager(robot_state_);

  // Create superstructure and associated commands.
  private final Superstructure superstructure_ = new Superstructure(turret_, shooter_, hood_,
      intake_, cargo_tracker_, robot_state_);

  private final Command score_tune_ = superstructure_.tuneScoring();
  private final Command score_lg_fender_ = superstructure_.scoreLowGoalFender();
  private final Command score_hg_fender_ = superstructure_.scoreHighGoalFender();
  private final Command score_hg_ = superstructure_.scoreHighGoal();

  // Create climber commands and tracker for climb mode.
  private final Command climb_auto_ = new ClimbAutomatic(climber_, driver_controller_::getAButton);
  private final Command climb_reset_ = new ClimbReset(climber_);
  private boolean climb_mode_ = false;

  // Create autonomous mode selector.
  private final SendableChooser<Command> auto_selector_ = new SendableChooser<>();
  private Command autonomous_command_ = null;

  // Keeps track of whether we need to clear buttons.
  private boolean clear_buttons_ = false;

  // Create telemetry.
  private final Telemetry telemetry_ = new Telemetry(
      robot_state_, drivetrain_, turret_, shooter_, hood_, intake_, climber_,
      superstructure_, cargo_tracker_, auto_selector_, () -> climb_mode_);

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

    // Update alliance color every second (just in case we lose comms or power).
    addPeriodic(() -> robot_state_.setAlliance(DriverStation.getAlliance()), 1);

    // Start command to zero the turret.
    new TurretZero(turret_, driver_controller_::getBackButton).schedule();
  }

  @Override
  public void disabledInit() {
    // Set coast mode on drivetrain, turret, and hood to make then easier to move.
    drivetrain_.setBrakeMode(false);
    turret_.setBrakeMode(false);
    hood_.setBrakeMode(false);
  }

  @Override
  public void autonomousInit() {
    // Set brake mode on turret and drivetrain.
    drivetrain_.setBrakeMode(true);
    turret_.setBrakeMode(true);
    hood_.setBrakeMode(true);

    // Set alliance color (guaranteed to be accurate here).
    DriverStation.Alliance alliance = DriverStation.getAlliance();
    robot_state_.setAlliance(alliance);

    // Start autonomous program.
    autonomous_command_ = auto_selector_.getSelected();
    if (autonomous_command_ != null) {
      autonomous_command_.schedule();
    }
  }

  @Override
  public void teleopInit() {
    // Set brake mode on drivetrain, turret, and hood.
    drivetrain_.setBrakeMode(true);
    turret_.setBrakeMode(true);
    hood_.setBrakeMode(true);

    climber_.enableSoftLimits(false);

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
    superstructure_.setTuning(score_tune_.isScheduled());

    // Limit drivetrain output if scoring.
    drivetrain_.limitOutput(score_hg_.isScheduled());

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
    auto_selector_.addOption("High Left 2",
        new HighLeft2(robot_state_, drivetrain_, superstructure_));
    auto_selector_.addOption("High Left 2 Steal 2",
        new HighLeft2Steal2(robot_state_, drivetrain_, superstructure_));
    auto_selector_.addOption("High Left 4",
        new HighLeft4(robot_state_, drivetrain_, superstructure_));

    auto_selector_.addOption("High Right 3",
        new HighRight3(robot_state_, drivetrain_, superstructure_));
    auto_selector_.addOption("High Right 3 Steal 1",
        new HighRight3Steal1(robot_state_, drivetrain_, superstructure_));
    auto_selector_.addOption("High Right 5",
        new HighRight5(robot_state_, drivetrain_, superstructure_));
  }

  /**
   * Sets default commands for each subsystem.
   */
  private void setDefaultCommands() {
    // Drivetrain:
    drivetrain_.setDefaultCommand(new DriveTeleop(drivetrain_, driver_controller_));

    // Climber:
    climber_.setDefaultCommand(new ClimbTeleop(climber_, driver_controller_, () -> climb_mode_));

    // Turret:
    turret_.setDefaultCommand(
        new SequentialCommandGroup(new WaitCommand(1), superstructure_.trackGoalWithTurret()));

    // Shooter:
    shooter_.setDefaultCommand(new RunCommand(() -> shooter_.setPercent(0), shooter_));

    // Hood:
    hood_.setDefaultCommand(superstructure_.trackGoalWithHood());

  }

  /**
   * Configures button / joystick bindings for teleop control (non-climb mode) if they are not
   * already configured in the respective subsystem default commands.
   */
  private void setupTeleopControls() {
    // A: none

    // B: climb mode toggle
    new Button(driver_controller_::getBButton).whenPressed(() -> {
      climb_mode_ = true;
      clear_buttons_ = true;
      new RunCommand(() -> turret_.setGoal(Math.toRadians(86), 0), turret_).schedule();
      new RunCommand(() -> hood_.setPosition(Hood.Constants.kMinAngle), hood_).schedule();
      intake_.setPivot(false);
    });

    // X: drivetrain cheesy drive quick turn (in command)

    // Y: none

    // LS: drivetrain movement (in command)

    // RS: none

    // LB: low goal fender preset
    new Button(driver_controller_::getLeftBumper)
        .whenHeld(score_lg_fender_)
        .whenPressed(new InstantCommand(robot_state_::resetPositionFromFender));

    // RB: high goal fender preset
    new Button(driver_controller_::getRightBumper)
        .whenHeld(score_hg_fender_)
        .whenPressed(new InstantCommand(robot_state_::resetPositionFromFender));

    // LT: intake
    new Button(() -> driver_controller_.getLeftTriggerAxis() > 0.1)
        .whenHeld(superstructure_.intake());

    // RT: score
    new Button(() -> driver_controller_.getRightTriggerAxis() > 0.1).whenHeld(score_hg_);

    // Back: eject
    new Button(driver_controller_::getBackButton).whenHeld(superstructure_.eject());
  }

  /**
   * Configures button / joystick bindings for endgame control (climb mode) if they are not
   * already configured in the respective subsystem default commands.
   */
  private void setupEndgameControls() {
    // A: toggle right arm (in command)

    // B: climb mode toggle
    new JoystickButton(driver_controller_, XboxController.Button.kB.value).whenPressed(() -> {
      climb_mode_ = false;
      clear_buttons_ = true;
      turret_.getDefaultCommand().schedule();
      hood_.getDefaultCommand().schedule();
    });

    // X: drivetrain cheesy drive quick turn (in command)

    // Y: toggle left arm (in command)

    // LS: drivetrain movement (in command)

    // RS: none

    // LB: left arm up (in command)

    // RB: right arm up (in command)

    // LT: left arm down (in command)

    // RT: right arm down (in command)

    // Back: reset and zero climber
    new JoystickButton(driver_controller_, XboxController.Button.kBack.value)
        .whenHeld(climb_reset_);

    // Start: toggle automatic climb
    new JoystickButton(driver_controller_, XboxController.Button.kStart.value)
        .toggleWhenPressed(climb_auto_);
  }

  /**
   * Updates the status of the LEDs periodically based on the various states of the robot (e.g.
   * climb mode, scoring, ball in intake).
   */
  private void updateLEDs() {
    if (climb_auto_.isScheduled()) {
      if (((ClimbAutomatic) climb_auto_).isWaiting())
        led_.setOutput(LED.OutputType.CLIMB_MODE_AUTOMATIC_WAITING);
      else
        led_.setOutput(LED.OutputType.CLIMB_MODE_AUTOMATIC);
    } else if (climb_reset_.isScheduled())
      led_.setOutput(LED.StandardLEDOutput.CLIMB_MODE_RESET);

    else if (climb_mode_)
      led_.setOutput(LED.StandardLEDOutput.CLIMB_MODE);

    else if (turret_.getStatus() == Turret.Status.NO_ZERO)
      led_.setOutput(LED.StandardLEDOutput.TURRET_NO_ZERO);

    else if (turret_.getStatus() == Turret.Status.ZEROING)
      led_.setOutput(LED.StandardLEDOutput.TURRET_ZEROING);

    else if (!limelight_manager_.isLimelightAlive())
      led_.setOutput(LED.OutputType.LIMELIGHT_ERROR);

    else if (isDisabled())
      led_.setOutput(LED.OutputType.RAINBOW);

    else if (score_tune_.isScheduled())
      led_.setOutput(LED.StandardLEDOutput.SCORING_TUNING);

    else if (score_hg_.isScheduled())
      led_.setOutput(LED.StandardLEDOutput.SCORING_AUTOMATIC);

    else if (score_hg_fender_.isScheduled() || score_lg_fender_.isScheduled())
      led_.setOutput(LED.StandardLEDOutput.SCORING_PRESET);

    else if (cargo_tracker_.getCargoCount() == 2)
      led_.setOutput(LED.StandardLEDOutput.ROBOT_STATE_TWO_BALL);

    else if (cargo_tracker_.getCargoCount() == 1)
      led_.setOutput(LED.StandardLEDOutput.ROBOT_STATE_ONE_BALL);

    else
      led_.setOutput(LED.StandardLEDOutput.BLANK);
  }
}
