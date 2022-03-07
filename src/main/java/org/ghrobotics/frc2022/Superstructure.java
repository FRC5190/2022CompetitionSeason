package org.ghrobotics.frc2022;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import java.io.IOException;
import java.util.Scanner;
import org.ghrobotics.frc2022.commands.FeederIndex;
import org.ghrobotics.frc2022.commands.FeederScore;
import org.ghrobotics.frc2022.commands.IntakePercent;
import org.ghrobotics.frc2022.subsystems.Feeder;
import org.ghrobotics.frc2022.subsystems.Hood;
import org.ghrobotics.frc2022.subsystems.Intake;
import org.ghrobotics.frc2022.subsystems.Shooter;
import org.ghrobotics.frc2022.subsystems.Turret;
import org.ghrobotics.frc2022.vision.GoalTracker;
import org.ghrobotics.lib.interpolation.LookupTable;

public class Superstructure {
  // Subsystems
  private final Turret turret_;
  private final Shooter shooter_;
  private final Hood hood_;
  private final Intake intake_;
  private final Feeder feeder_;

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

    // Assign goal tracker.
    goal_tracker_ = goal_tracker;

    // Assign robot state.
    robot_state_ = robot_state;

    double x = HighGoalLUT.getHoodAngle(0);
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
    Translation2d goal = goal_tracker_.getClosestTarget(robot_pose_).getTranslation();
    /* Translation2d goal = Constants.kGoal; */ // pose estimator

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
        new FeederScore(feeder_, shooter_::atGoal)
    );
  }

  /**
   * Returns the command to score cargo into the high goal.
   *
   * @return The command to score cargo into the high goal.
   */
  public Command scoreHighGoal() {
    // The following commands run in parallel:
    //  - set turret angle, shooter speed, hood angle
    //  - wait for spin up and score
    return new ParallelCommandGroup(
        new RunCommand(() -> {
          // Calculate turret angular velocity setpoint.
          double turret_omega = -robot_speeds_.omegaRadiansPerSecond -
              robot_speeds_.vxMetersPerSecond * Math.sin(robot_to_goal_angle_) /
                  robot_to_goal_distance_;

          // Calculate shooter speed and hood angle from lookup table.
          double shooter_speed = HighGoalLUT.getShooterSpeed(robot_to_goal_distance_);
          double hood_angle = HighGoalLUT.getShooterSpeed(robot_to_goal_distance_);

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
        new FeederScore(feeder_, shooter_::atGoal)
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
        () -> hood_.setPosition(HighGoalLUT.getHoodAngle(robot_to_goal_distance_)), hood_);
  }

  public static class HighGoalLUT {
    private static final LookupTable rpm_lut_;
    private static final LookupTable hood_angle_lut_;

    static {
      rpm_lut_ = new LookupTable(3000);
      hood_angle_lut_ = new LookupTable(5);

      // Read table from filesystem.
      try {
        Scanner scanner = new Scanner(
            Filesystem.getDeployDirectory().toPath().resolve("shooter_lut.csv"));

        while (scanner.hasNextLine()) {
          String[] str_values = scanner.nextLine().split(",");
          double distance = Double.parseDouble(str_values[0]);
          double rpm = Double.parseDouble(str_values[1]);
          double angle = Double.parseDouble(str_values[2]);

          rpm_lut_.put(distance, rpm);
          hood_angle_lut_.put(distance, 90 - angle);
          System.out.printf("Added entry to LUT -> Distance: %2.2f m: %4f rpm, %2f deg\n", distance,
              rpm, angle);
        }

      } catch (IOException e) {
        e.printStackTrace();
      }
    }

    /**
     * Returns the shooter speed to score in the high goal.
     *
     * @param distance The distance to the goal.
     * @return The shooter speed to score in the high goal.
     */
    public static double getShooterSpeed(double distance) {
      return rpm_lut_.get(distance);
    }

    /**
     * The hood angle to score in the high goal.
     *
     * @param distance The distance to the goal.
     * @return The hood angle to score in the high goal.
     */
    public static double getHoodAngle(double distance) {
      return Math.toRadians(hood_angle_lut_.get(distance));
    }
  }

  public static class Constants {
    // Goal
    public static final Translation2d kGoal = new Translation2d(
        Units.feetToMeters(27), Units.feetToMeters(13.5));

    // Low Goal Scoring
    public static final double kLowGoalHoodAngle = Math.toRadians(40);
    public static final double kLowGoalShooterRPM = 3000;

    // Intake
    public static final double kIntakeCollectSpeed = +0.85;
  }
}
