package org.ghrobotics.frc2022;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.function.BooleanSupplier;
import org.ghrobotics.frc2022.commands.IntakeAutomatic;
import org.ghrobotics.frc2022.planners.HighGoalPlanner;
import org.ghrobotics.frc2022.subsystems.CargoTracker;
import org.ghrobotics.frc2022.subsystems.Hood;
import org.ghrobotics.frc2022.subsystems.Intake;
import org.ghrobotics.frc2022.subsystems.LimelightManager;
import org.ghrobotics.frc2022.subsystems.Shooter;
import org.ghrobotics.frc2022.subsystems.Turret;

public class Superstructure {
  // Subsystems
  private final Turret turret_;
  private final Shooter shooter_;
  private final Hood hood_;
  private final Intake intake_;

  // High Goal Planner
  private final HighGoalPlanner high_goal_planner_;

  // Cargo Tracker and Robot State
  private final CargoTracker cargo_tracker_;
  private final RobotState robot_state_;

  // Goal
  private final BooleanSupplier ready_to_score_;

  // Tracking
  private Pose2d robot_pose_;
  private ChassisSpeeds robot_speeds_;
  private Translation2d turret_to_goal_;
  private double turret_to_goal_distance_;
  private double turret_to_goal_angle_;

  private boolean turret_eject_;
  private double turret_to_eject_angle_;

  // Tuning Mode
  private boolean tuning_;

  // Timer
  private final Timer timer_;

  /**
   * This class manages all mechanisms on the "superstructure" i.e. turret, shooter, hood, intake,
   * and feeder. It is responsible for interactions between these subsystems while scoring,
   * intaking, etc.
   *
   * @param turret  Reference to turret subsystem.
   * @param shooter Reference to shooter subsystem.
   * @param hood    Reference to hood subsystem.
   * @param intake  Reference to intake subsystem.
   */
  public Superstructure(Turret turret, Shooter shooter, Hood hood, Intake intake,
                        CargoTracker cargo_tracker, RobotState robot_state) {
    // Assign subsystem references.
    turret_ = turret;
    shooter_ = shooter;
    hood_ = hood;
    intake_ = intake;

    // Initialize high goal planner.
    high_goal_planner_ = new HighGoalPlanner(Constants.kHighGoalShooterRPM,
        Constants.kHighGoalHoodAngle);

    // Assign cargo tracker and robot state.
    cargo_tracker_ = cargo_tracker;
    robot_state_ = robot_state;

    // Initialize timer.
    timer_ = new Timer();

    // Create ready-to-score supplier.
    ready_to_score_ = () -> (shooter_.atGoal() && turret_.atGoal() && hood_.atGoal()) ||
        timer_.get() > 2.5;
  }

  /**
   * This should be run periodically every 20 ms to update the local variables used for storing
   * the robot state, the latest goal pose, etc.
   */
  public void periodic() {
    // Get the latest robot pose and speeds.
    robot_pose_ = robot_state_.getRobotPose();
    robot_speeds_ = robot_state_.getRobotSpeeds();

    // Get the current alliance color and the color of the next ball to shoot.
    DriverStation.Alliance alliance = robot_state_.getAlliance();
    Color next_cargo_color = cargo_tracker_.getNextBall();

    // If the alliance is invalid or there is no next ball or the alliance color matches the
    // color of the next ball, do not eject.
    turret_eject_ = !(alliance == DriverStation.Alliance.Invalid ||
        next_cargo_color == null ||
        alliance == DriverStation.Alliance.Red && next_cargo_color.red > 0.9 ||
        alliance == DriverStation.Alliance.Blue && next_cargo_color.blue > 0.9);

    // Calculate turret pose.
    Pose2d turret_pose = robot_pose_.transformBy(
        new Transform2d(new Translation2d(LimelightManager.Constants.kRobotToTurretDistance, 0),
            new Rotation2d()));

    // Calculate translation to goal.
    turret_to_goal_ = new Pose2d(Arena.kGoal, new Rotation2d())
        .relativeTo(turret_pose).getTranslation();

    // Calculate distance and angle to goal.
    turret_to_goal_distance_ = turret_to_goal_.getNorm();
    turret_to_goal_angle_ = Math.atan2(turret_to_goal_.getY(), turret_to_goal_.getX());

    // Calculate turret angle to eject.
    Translation2d turret_to_hangar = new Pose2d(Arena.kHangar, new Rotation2d())
        .relativeTo(turret_pose).getTranslation();
    turret_to_eject_angle_ = Math.atan2(turret_to_hangar.getY(), turret_to_hangar.getX());

    // Add debug values to SmartDashboard (if not in tuning mode).
    if (!tuning_) {
      SmartDashboard.putNumber(Constants.kTuningShooterRPMKey,
          Units.radiansPerSecondToRotationsPerMinute(
              high_goal_planner_.getShooterSpeed(turret_to_goal_distance_)));
      SmartDashboard.putNumber(Constants.kTuningHoodAngleKey,
          Math.toDegrees(
              high_goal_planner_.getHoodAngle(turret_to_goal_distance_)));
    }
  }

  /**
   * Returns the command to intake balls and store in the feeder.
   *
   * @return The command to intake balls and store in the feeder.
   */
  public Command intake() {
    return new IntakeAutomatic(intake_, cargo_tracker_, () -> true, () -> false);
  }

  /**
   * Returns the command to score cargo into the low goal from the fender.
   *
   * @return The command to score cargo into the low goal from the fender.
   */
  public Command scoreLowGoalFender() {
    // The following commands run in parallel:
    //  - set turret to 180 deg
    //  - set shooter speed and hood angle to preset values
    //  - wait for spin up and score
    return new ParallelCommandGroup(
        startTimer(),
        new RunCommand(() -> turret_.setGoal(Math.toRadians(180), 0), turret_),
        new RunCommand(() -> shooter_.setRPM(Constants.kLowGoalShooterRPM), shooter_),
        new RunCommand(() -> hood_.setPosition(Constants.kLowGoalHoodAngle), hood_),
        new IntakeAutomatic(intake_, cargo_tracker_, () -> false, ready_to_score_)
    );
  }

  /**
   * Returns the command to score cargo into the high goal from the fender.
   *
   * @return The command to score cargo into the high goal from the fender.
   */
  public Command scoreHighGoalFender() {
    // The following commands run in parallel:
    //  - set turret to 180 deg
    //  - set shooter speed and hood angle to preset values
    //  - wait for spin up and score
    return new ParallelCommandGroup(
        startTimer(),
        new RunCommand(() -> turret_.setGoal(Math.toRadians(180), 0), turret_),
        new RunCommand(() -> shooter_.setRPM(Constants.kHighGoalShooterRPM), shooter_),
        new RunCommand(() -> hood_.setPosition(Constants.kHighGoalHoodAngle), hood_),
        new IntakeAutomatic(intake_, cargo_tracker_, () -> false, ready_to_score_)
    );
  }

  /**
   * Returns the command to score cargo into the high goal.
   *
   * @param require_intake Whether the intake should also be spinning while scoring (enables
   *                       shooting while moving while intaking).
   * @param wait_for_score When scoring should occur; when the supplier returns true and the
   *                       shooter is at the goal, scoring will occur.
   * @return The command to score cargo into the high goal.
   */
  public Command scoreHighGoal(BooleanSupplier require_intake, BooleanSupplier wait_for_score) {
    BooleanSupplier score = () -> ready_to_score_.getAsBoolean() && wait_for_score.getAsBoolean();

    // The following commands run in parallel:
    //  - set turret angle, shooter speed, hood angle
    //  - intake (if required)
    //  - wait for spin up and score
    return new ParallelCommandGroup(
        startTimer(),
        new RunCommand(() -> {
          // Calculate turret angular velocity setpoint.
          double turret_omega = -robot_speeds_.omegaRadiansPerSecond -
              robot_speeds_.vxMetersPerSecond * Math.sin(turret_to_goal_angle_) /
                  turret_to_goal_distance_;

          // Calculate shooter speed and hood angle from lookup table.
          double shooter_speed = high_goal_planner_.getShooterSpeed(turret_to_goal_distance_);
          double hood_angle = high_goal_planner_.getHoodAngle(turret_to_goal_distance_);

          // Get the speed of the ball on the xy plane.
          double ball_vxy = shooter_speed * Shooter.Constants.kWheelRadius / 2 *
              Math.sin(hood_angle);

          // Calculate time of flight (assuming no air resistance).
          double t = turret_to_goal_distance_ / ball_vxy;

          // Calculate new distance to goal, assuming same time of flight.
          double adjusted_distance = Math.sqrt(
              Math.pow(turret_to_goal_distance_, 2) + Math.pow(robot_speeds_.vxMetersPerSecond, 2) -
                  2 * turret_to_goal_distance_ * robot_speeds_.vxMetersPerSecond * t *
                      Math.cos(turret_to_goal_angle_)
          );

          // Calculate turret angle.
          double turret_theta = turret_to_goal_angle_ +
              Math.asin(robot_speeds_.vxMetersPerSecond * t * Math.sin(turret_to_goal_angle_) /
                  adjusted_distance);

          // Use adjusted distance to calculate new shooter speed.
          shooter_speed = (adjusted_distance / t) / Math.sin(hood_angle) /
              Shooter.Constants.kWheelRadius * 2;

          // Set goals to subsystems.
          if (turret_eject_) {
            // If we have the wrong colored ball coming up, point toward hangar and use low goal
            // speeds and angles.
            turret_.setGoal(turret_to_eject_angle_, 0);
            shooter_.setRPM(Constants.kLowGoalShooterRPM);
            hood_.setPosition(Constants.kLowGoalHoodAngle);
          } else {
            // If we don't need to eject the ball, use calculations from above.
            turret_.setGoal(turret_theta, turret_omega);
            shooter_.setVelocity(shooter_speed);
            hood_.setPosition(hood_angle);
          }
        }, turret_, shooter_, hood_),
        new IntakeAutomatic(intake_, cargo_tracker_, require_intake, score)
    ).withInterrupt(() -> cargo_tracker_.getCargoCount() == 0);
  }

  /**
   * Returns the command to score cargo into the high goal (with no intake).
   *
   * @return The command to score cargo into the high goal (with no intake).
   */
  public Command scoreHighGoal() {
    return scoreHighGoal(() -> false, () -> true);
  }

  /**
   * Returns the command to tune the shooter and hood for building the lookup table. The building
   * of the lookup table still needs to be done manually.
   *
   * @return The command to tune the shooter and hood for building the lookup table.
   */
  public Command tuneScoring() {
    // The following commands run in parallel:
    //  - run shooter at desired speed
    //  - run hood at desired position
    //  - track goal with turret
    //  - run intake and feeder
    return new ParallelCommandGroup(
        new RunCommand(() -> shooter_.setRPM(
            SmartDashboard.getNumber(Constants.kTuningShooterRPMKey, 0)), shooter_),
        new RunCommand(() -> hood_.setPosition(
            Math.toRadians(SmartDashboard.getNumber(Constants.kTuningHoodAngleKey, 0))), hood_),
        new IntakeAutomatic(intake_, cargo_tracker_, () -> true, () -> true),
        trackGoalWithTurret()
    );
  }


  /**
   * Returns the command to continuously track the goal with the turret.
   *
   * @return The command to continuously track the goal with the turret.
   */
  public Command trackGoalWithTurret() {
    return new RunCommand(() -> turret_.setGoal(turret_to_goal_angle_,
        -robot_speeds_.omegaRadiansPerSecond -
            robot_speeds_.vxMetersPerSecond * Math.sin(turret_to_goal_angle_) /
                turret_to_goal_distance_), turret_);
  }

  /**
   * Returns the command to continuously track the goal with the hood.
   *
   * @return The command to continuously track the goal with the hood.
   */
  public Command trackGoalWithHood() {
    return new RunCommand(
        () -> {
          // Check whether we are out of the safe hangar zone.
          if (robot_pose_.getX() > Arena.kSafeHangarTLCorner.getX() &&
              robot_pose_.getY() < Arena.kSafeHangarTLCorner.getY() &&
              robot_pose_.getX() < Arena.kSafeHangarBRCorner.getX() &&
              robot_pose_.getY() > Arena.kSafeHangarBRCorner.getY()) {
            hood_.setPosition(Hood.Constants.kMinAngle);
          } else {
            hood_.setPosition(high_goal_planner_.getHoodAngle(turret_to_goal_distance_));
          }
        }, hood_);
  }

  /**
   * Sets whether we are currently tuning the superstructure.
   *
   * @param value Whether we are currently tuning the superstructure.
   */
  public void setTuning(boolean value) {
    tuning_ = value;
  }

  /**
   * Returns the distance from the robot to goal.
   *
   * @return The distance from the robot to goal.
   */
  public double getRobotToGoalDistance() {
    return turret_to_goal_distance_;
  }

  /**
   * Returns the angle from the robot to goal.
   *
   * @return The angle from the robot to goal.
   */
  public double getRobotToGoalAngle() {
    return turret_to_goal_angle_;
  }

  /**
   * Returns the number of cargo in the superstructure.
   *
   * @return The number of cargo in the superstructure.
   */
  public int getCargoCount() {
    return cargo_tracker_.getCargoCount();
  }

  /**
   * Returns the command to start a timer from the time the button was pressed.
   *
   * @return The command to start a timer from the time the button was pressed.
   */
  private Command startTimer() {
    return new InstantCommand(() -> {
      // Stop, reset, and start timer.
      timer_.stop();
      timer_.reset();
      timer_.start();
    });
  }

  public static class Constants {
    // Low Goal Scoring
    public static final double kLowGoalHoodAngle = Math.toRadians(42);
    public static final double kLowGoalShooterRPM = 1200;

    // High Goal Scoring
    public static final double kHighGoalHoodAngle = Math.toRadians(13);
    public static final double kHighGoalShooterRPM = 2600;

    // Tuning Entries
    public static final String kTuningShooterRPMKey = "Shooter Speed (rpm)";
    public static final String kTuningHoodAngleKey = "Hood Angle (deg)";
    public static final String kTuningScoreKey = "Score";
  }
}
