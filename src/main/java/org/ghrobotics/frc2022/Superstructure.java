package org.ghrobotics.frc2022;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.concurrent.atomic.AtomicReference;
import org.ghrobotics.frc2022.commands.FeederIndex;
import org.ghrobotics.frc2022.commands.FeederPercent;
import org.ghrobotics.frc2022.commands.FeederScore;
import org.ghrobotics.frc2022.commands.IntakePercent;
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

  // Goal Tracker
  private final GoalTracker goal_tracker_;

  // Robot State
  private final RobotState robot_state_;

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
   * Returns the command to exhaust balls out of the feeder and through the intake.
   *
   * @return The command to exhaust balls out of the feeder and through the intake.
   */
  public Command exhaust() {
    // The following commands run in parallel:
    //  - the command to run the feeder in reverse
    //  - the command to run the intake in reverse
    return new ParallelCommandGroup(
        new IntakePercent(intake_, Constants.kIntakeExhaustSpeed),
        new FeederPercent(feeder_, Constants.kFeederExhaustSpeed, Constants.kFeederExhaustSpeed));
  }

  /**
   * Returns the command to score cargo into the hub.
   *
   * @param high_goal Whether the cargo should be scored into the high goal (low goal otherwise).
   * @return The command to score cargo into the hub.
   */
  public Command scoreCargo(boolean high_goal) {
    // Store goal pose.
    AtomicReference<Pose2d> goal_pose = new AtomicReference<>();

    // The following commands run sequentially:
    //  - store goal pose.
    //  - score
    return new SequentialCommandGroup(
        new InstantCommand(() -> goal_pose.set(goal_tracker_.getClosestTarget(
            robot_state_.getRobotPose()))),

        // The following commands run in parallel:
        //  - the command to track the goal with the turret (incl. shooting while moving).
        //  - the command to track the goal with the hood.
        //  - the command to run the shooter at the desired rpm (incl. shooting while moving).
        //  - the command to run the feeder into the shooter.
        new ParallelCommandGroup(
            new RunCommand(() -> {
              // Get robot pose and speeds.
              Pose2d robot_pose = robot_state_.getRobotPose();
              ChassisSpeeds robot_speeds = robot_state_.getRobotSpeeds();

              // Get goal pose.
              Pose2d goal = goal_pose.get();

              // Get turret setpoints assuming stationary robot.
              double[] turret_setpoints = calculateTurretThetaOmega(robot_pose, robot_speeds, goal);

              // Break out into turret position and velocity.
              double turret_pos = turret_setpoints[0];
              double turret_vel = turret_setpoints[1];

              // Calculate distance to goal.
              double distance = robot_pose.getTranslation().getDistance(goal.getTranslation());

              // Obtain shooter speed and hood angle setpoints from the lookup table.
              double shooter_speed = high_goal ? HighGoalLUT.getShooterSpeed(distance) :
                  LowGoalLUT.getShooterSpeed(distance);
              double hood_angle = high_goal ? HighGoalLUT.getHoodAngle(distance) :
                  LowGoalLUT.getHoodAngle(distance);

              // Get the speed of the ball on the xy plane.
              double ball_xy = shooter_speed * Shooter.Constants.kWheelRadius / 2 *
                  Math.sin(hood_angle);

              // Calculate time of flight (assuming no air resistance).
              double t = distance / ball_xy;

              // Calculate new distance to goal, assuming same time of flight.
              // TODO: Finish Math
            }),
            new FeederScore(feeder_, shooter_::atGoal))
    );
  }


  /**
   * Returns the command to continuously track the goal with the turret.
   *
   * @return The command to continuously track the goal with the turret.
   */
  public Command trackGoalWithTurret() {
    return new RunCommand(() -> {
      // Get desired turret position and velocity.
      double[] turret_setpoints = calculateTurretThetaOmega(
          robot_state_.getRobotPose(),
          robot_state_.getRobotSpeeds(),
          goal_tracker_.getClosestTarget(robot_state_.getRobotPose()));

      // Set turret goal.
      turret_.setGoal(turret_setpoints[0], turret_setpoints[1]);
    }, turret_);
  }

  /**
   * Returns the command to continuously track the goal with the hood.
   *
   * @return The command to continuously track the goal with the hood.
   */
  public Command trackGoalWithHood() {
    return new InstantCommand();
  }

  /**
   * Calculates the desired turret position and velocity to track the goal.
   *
   * @param robot_pose   The robot pose.
   * @param robot_speeds The robot speeds.
   * @param goal_pose    The goal pose.
   * @return The desired turret position and velocity to track the goal.
   */
  private static double[] calculateTurretThetaOmega(Pose2d robot_pose, ChassisSpeeds robot_speeds,
                                                    Pose2d goal_pose) {
    // Calculate translation from robot pose to goal pose.
    Translation2d robot_to_goal = goal_pose.getTranslation().minus(robot_pose.getTranslation());

    // Calculate distance and angle to goal.
    double distance_to_goal = robot_to_goal.getNorm();
    double angle_to_goal = Math.atan2(robot_to_goal.getY(), robot_to_goal.getX()) -
        robot_pose.getRotation().getRadians();

    // Calculate angular velocity of the robot about the goal (v = r * omega).
    double angular_velocity_about_goal = robot_speeds.vxMetersPerSecond *
        Math.sin(angle_to_goal) / distance_to_goal;

    // Calculate turret velocity.
    double turret_omega = -angular_velocity_about_goal - robot_speeds.omegaRadiansPerSecond;

    // Return position and velocity.
    return new double[]{angle_to_goal, turret_omega};
  }

  public static class HighGoalLUT {
    /**
     * Returns the shooter speed to score in the high goal.
     *
     * @param distance The distance to the goal.
     * @return The shooter speed to score in the high goal.
     */
    public static double getShooterSpeed(double distance) {
      return 0;
    }

    /**
     * The hood angle to score in the high goal.
     *
     * @param distance The distance to the goal.
     * @return The hood angle to score in the high goal.
     */
    public static double getHoodAngle(double distance) {
      return 0;
    }
  }

  public static class LowGoalLUT {
    /**
     * Returns the shooter speed to score in the low goal.
     *
     * @param distance The distance to the goal.
     * @return The shooter speed to score in the low goal.
     */
    public static double getShooterSpeed(double distance) {
      return 0;
    }

    /**
     * The hood angle to score in the low goal.
     *
     * @param distance The distance to the goal.
     * @return The hood angle to score in the low goal.
     */
    public static double getHoodAngle(double distance) {
      return 0;
    }
  }

  public static class Constants {
    // Intake
    public static final double kIntakeCollectSpeed = +0.85;
    public static final double kIntakeExhaustSpeed = -0.85;

    // Feeder
    public static final double kFeederExhaustSpeed = -0.75;
  }
}
