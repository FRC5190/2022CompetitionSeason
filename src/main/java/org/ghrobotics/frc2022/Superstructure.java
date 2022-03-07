package org.ghrobotics.frc2022;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import java.util.function.BooleanSupplier;
import org.ghrobotics.frc2022.commands.FeederIndex;
import org.ghrobotics.frc2022.commands.IntakePercent;
import org.ghrobotics.frc2022.planners.HighGoalPlanner;
import org.ghrobotics.frc2022.subsystems.Feeder;
import org.ghrobotics.frc2022.subsystems.Hood;
import org.ghrobotics.frc2022.subsystems.Intake;
import org.ghrobotics.frc2022.subsystems.Shooter;
import org.ghrobotics.frc2022.subsystems.Turret;
import org.ghrobotics.frc2022.vision.GoalTracker;

public class Superstructure {
  // Subsystems
  private final Turret turret_;
  private final Shooter shooter_;
  private final Hood hood_;
  private final Intake intake_;
  private final Feeder feeder_;

  // High Goal Planner
  private final HighGoalPlanner high_goal_planner_;

  // Goal Tracker
  private final GoalTracker goal_tracker_;

  // Robot State
  private final RobotState robot_state_;

  // Tracking
  private Pose2d robot_pose_;
  private ChassisSpeeds robot_speeds_;
  private Translation2d robot_to_goal_;
  private double robot_to_goal_distance_;
  private double robot_to_goal_angle_;

  // Status
  private boolean scoring_ = false;


  /**
   * This class manages all mechanisms on the "superstructure" i.e. turret, shooter, hood, intake,
   * and feeder. It is responsible for interactions between these subsystems while scoring,
   * intaking, etc.
   *
   * @param turret  Reference to turret subsystem.
   * @param shooter Reference to shooter subsystem.
   * @param hood    Reference to hood subsystem.
   * @param intake  Reference to intake subsystem.
   * @param feeder  Reference to feeder subsystem.
   */
  public Superstructure(Turret turret, Shooter shooter, Hood hood, Intake intake, Feeder feeder,
                        GoalTracker goal_tracker, RobotState robot_state) {
    // Assign subsystem references.
    turret_ = turret;
    shooter_ = shooter;
    hood_ = hood;
    intake_ = intake;
    feeder_ = feeder;

    // Initialize high goal planner.
    high_goal_planner_ = new HighGoalPlanner(Constants.kLowGoalShooterRPM,
        Constants.kLowGoalHoodAngle);

    // Assign goal tracker.
    goal_tracker_ = goal_tracker;

    // Assign robot state.
    robot_state_ = robot_state;
  }

  /**
   * This should be run periodically every 20 ms to update the local variables used for storing
   * the robot state, the latest goal pose, etc.
   */
  public void periodic() {
    // Get the latest robot pose and speeds.
    robot_pose_ = robot_state_.getRobotPose();
    robot_speeds_ = robot_state_.getRobotSpeeds();

    // Get the goal position. (We can swap between GoalTracker and Pose Estimator here).
//    Translation2d goal = goal_tracker_.getClosestTarget(robot_pose_).getTranslation();
    Translation2d goal = Constants.kGoal; // pose estimator

    // Calculate translation to goal.
    robot_to_goal_ = goal.minus(robot_pose_.getTranslation());

    // Calculate distance and angle to goal.
    robot_to_goal_distance_ = robot_to_goal_.getNorm();
    robot_to_goal_angle_ = Math.atan2(robot_to_goal_.getY(), robot_to_goal_.getX())
        - robot_pose_.getRotation().getRadians();
  }

  /**
   * Returns the command to intake balls and store in the feeder.
   *
   * @return The command to intake balls and store in the feeder.
   */
  public Command intake() {
    // The following commands run in parallel:
    //  - command to run the intake at the desired speed.
    //  - command to run the feeder based on sensor input.
    return new ParallelCommandGroup(
        new IntakePercent(intake_, Constants.kIntakeCollectSpeed),
        new FeederIndex(feeder_));
  }

  /**
   * Returns the command to score cargo into the low goal.
   *
   * @return The command to score cargo into the low goal.
   */
  public Command scoreLowGoal() {
    // The following commands run in parallel:
    //  - set turret to 180 deg
    //  - set shooter speed and hood angle to preset values
    //  - wait for spin up and score
    return new ParallelCommandGroup(
        new RunCommand(() -> turret_.setGoal(Math.toRadians(180), 0), turret_),
        new RunCommand(() -> shooter_.setRPM(Constants.kLowGoalShooterRPM), shooter_),
        new RunCommand(() -> hood_.setPosition(Constants.kLowGoalHoodAngle), hood_),
        new FeederIndex(feeder_, shooter_::atGoal)
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
  public Command scoreHighGoal(boolean require_intake, BooleanSupplier wait_for_score) {
    BooleanSupplier score = () -> shooter_.atGoal() && wait_for_score.getAsBoolean();

    // The following commands run in parallel:
    //  - set turret angle, shooter speed, hood angle
    //  - intake (if required)
    //  - wait for spin up and score
    return new ParallelCommandGroup(
        new RunCommand(() -> {
          // Calculate turret angular velocity setpoint.
          double turret_omega = -robot_speeds_.omegaRadiansPerSecond -
              robot_speeds_.vxMetersPerSecond * Math.sin(robot_to_goal_angle_) /
                  robot_to_goal_distance_;

          // Calculate shooter speed and hood angle from lookup table.
          double shooter_speed = high_goal_planner_.getShooterSpeed(robot_to_goal_distance_);
          double hood_angle = high_goal_planner_.getHoodAngle(robot_to_goal_distance_);

          // Get the speed of the ball on the xy plane.
          double ball_vxy = shooter_speed * Shooter.Constants.kWheelRadius / 2 *
              Math.sin(hood_angle);

          // Calculate time of flight (assuming no air resistance).
          double t = robot_to_goal_distance_ / ball_vxy;

          // Calculate new distance to goal, assuming same time of flight.
          double adjusted_distance = Math.sqrt(
              Math.pow(robot_to_goal_distance_, 2) + Math.pow(robot_speeds_.vxMetersPerSecond, 2) -
                  2 * robot_to_goal_distance_ * robot_speeds_.vxMetersPerSecond * t *
                      Math.cos(robot_to_goal_angle_)
          );

          // Calculate turret angle.
          double turret_theta = robot_to_goal_angle_ +
              Math.asin(robot_speeds_.vxMetersPerSecond * t * Math.sin(robot_to_goal_angle_) /
                  adjusted_distance);

          // Use adjusted distance to calculate new shooter speed.
          shooter_speed = (adjusted_distance / t) / Math.sin(hood_angle) /
              Shooter.Constants.kWheelRadius * 2;

          // Set goals to subsystems.
          turret_.setGoal(turret_theta, turret_omega);
          shooter_.setVelocity(shooter_speed);
          hood_.setPosition(hood_angle);
        }, turret_, shooter_, hood_),
        new FeederIndex(feeder_, score),
        require_intake ? new IntakePercent(intake_,
            Constants.kIntakeCollectSpeed) : new InstantCommand()
    );
  }

  /**
   * Returns the command to score cargo into the high goal (with no intake).
   *
   * @return The command to score cargo into the high goal (with no intake).
   */
  public Command scoreHighGoal() {
    return scoreHighGoal(false, () -> true);
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
        new RunCommand(() -> shooter_.setVelocity(
            SmartDashboard.getNumber(Constants.kTuningShooterRPMKey, Constants.kLowGoalShooterRPM)),
            shooter_),
        new RunCommand(() -> hood_.setPosition(
            SmartDashboard.getNumber(Constants.kTuningHoodAngleKey, Constants.kLowGoalHoodAngle)),
            hood_),
        new IntakePercent(intake_, Constants.kIntakeCollectSpeed),
        new FeederIndex(feeder_, () -> SmartDashboard.getBoolean(Constants.kTuningScoreKey, false)),
        trackGoalWithTurret()
    );
  }


  /**
   * Returns the command to continuously track the goal with the turret.
   *
   * @return The command to continuously track the goal with the turret.
   */
  public Command trackGoalWithTurret() {
    return new RunCommand(() -> turret_.setGoal(robot_to_goal_angle_,
        -robot_speeds_.omegaRadiansPerSecond -
            robot_speeds_.vxMetersPerSecond * Math.sin(robot_to_goal_angle_) /
                robot_to_goal_distance_), turret_);
  }

  /**
   * Returns the command to continuously track the goal with the hood.
   *
   * @return The command to continuously track the goal with the hood.
   */
  public Command trackGoalWithHood() {
    return new RunCommand(
        () -> hood_.setPosition(high_goal_planner_.getHoodAngle(robot_to_goal_distance_)), hood_);
  }

  public static class Constants {
    // Goal
    public static final Translation2d kGoal = new Translation2d(
        Units.feetToMeters(27), Units.feetToMeters(13.5));

    // Low Goal Scoring
    public static final double kLowGoalHoodAngle = Math.toRadians(40);
    public static final double kLowGoalShooterRPM = 3000;

    // Intake
    public static final double kIntakeCollectSpeed = 0.85;

    // Tuning Entries
    public static final String kTuningShooterRPMKey = "Shooter Speed (rpm)";
    public static final String kTuningHoodAngleKey = "Hood Angle (deg)";
    public static final String kTuningScoreKey = "Score";
  }
}
