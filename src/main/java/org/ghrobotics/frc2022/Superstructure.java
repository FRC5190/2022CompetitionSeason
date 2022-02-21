package org.ghrobotics.frc2022;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.ghrobotics.frc2022.vision.Limelight;
import static org.ghrobotics.frc2022.Superstructure.Constants.kGoalLocation;

/**
 * This class manages all interactions between subsystems. It uses state machines to avoid
 * mechanism collisions, setup target tracking, etc.
 */
public class Superstructure {
  // TODO: add drivetrain, turret, shooter, hood, intake, feeder, climber

  // Limelight
  private final Limelight limelight_;

  // State Machines
  private DrivetrainState drivetrain_state_ = DrivetrainState.MANUAL;
  private ScoringState scoring_state_ = ScoringState.STOW;
  private CollectionState collection_state_ = CollectionState.RETRACT;
  private ClimbState climb_state_ = ClimbState.STOW;

  /**
   * Initializes the superstructure and all robot subsystems.
   */
  public Superstructure() {
    // TODO: initialize subsystems

    // Initialize Limelight.
    limelight_ = new Limelight("limelight");
  }

  /**
   * This method should be called periodically to update the superstructure state machine. Here,
   * the goal for each subsystem is set.
   *
   * @param timestamp The current time.
   */
  public void update(double timestamp) {
    // VISION
    // Check if we are tracking any targets at this time.
    if (limelight_.hasTarget()) {
      // Calculate pitch and height of Limelight (depends on hood angle).
      // TODO
      Rotation2d limelight_pitch = new Rotation2d();
      double limelight_height = 0;

      // Obtain target information.
      double image_capture_timestamp = timestamp - limelight_.getLatency();
      Rotation2d target_yaw = limelight_.getTx();
      Rotation2d target_pitch = limelight_.getTy();

      // Calculate distance to target.
      // https://www.chiefdelphi.com/t/calculating-distance-to-vision-target/387183/6?u=prateek_m
      double target_distance = (Constants.kTargetHeight - limelight_height) /
          target_yaw.getCos() / target_pitch.plus(limelight_pitch).getTan();

      // Calculate robot pose from distance and yaw to target.
      // TODO

      // Add robot pose to pose estimator.
      // TODO
    }

    // DRIVETRAIN
    switch (drivetrain_state_) {
      case PATH_FOLLOW:
        break;
      case MANUAL:
        break;
    }

    // CLIMBER
    switch (climb_state_) {
      case CLIMB_L4:
        break;
      case READY_FOR_L4:
        break;
      case CLIMB_L3:
        break;
      case READY_FOR_L3:
        break;
      case CLIMB_L2:
        break;
      case READY_FOR_L2:
        break;
      case STOW:
        break;
    }

    // TURRET / SHOOTER / HOOD
    switch (scoring_state_) {
      case AUTOMATIC_HIGH_GOAL:
        break;
      case MANUAL_HIGH_GOAL:
        break;
      case LOW_GOAL:
        break;
      case AUTOMATIC_TRACK:
        break;
      case MANUAL_TRACK:
        break;
      case STOW:
        break;
    }

    // INTAKE / FEEDER
    switch (collection_state_) {
      case FEED:
        break;
      case COLLECT:
        break;
      case INTAKE:
        break;
      case IDLE:
        break;
      case RETRACT:
        break;
    }
  }

  /**
   * Calculates the angular velocity of the turret to keep track of the goal as the robot moves
   * around the goal.
   *
   * @param chassis_pose   The position of the robot.
   * @param chassis_speeds The speed of the robot.
   * @return The angular velocity of the turret (in rad/s).
   */
  private static double calculateTurretOmega(Pose2d chassis_pose, ChassisSpeeds chassis_speeds) {
    // We need to counteract the drivetrain's rotation:
    double angular_component = -chassis_speeds.omegaRadiansPerSecond;

    // Calculate component from robot's tangential motion around the goal:
    // Find the translation from the robot to the goal.
    Translation2d robot_to_goal = kGoalLocation.minus(chassis_pose.getTranslation());

    // Compute the angle and distance to the goal.
    Rotation2d angle_to_goal = new Rotation2d(robot_to_goal.getX(), robot_to_goal.getY())
        .minus(chassis_pose.getRotation());
    double distance_to_goal = robot_to_goal.getNorm();

    // Compute the velocity vector of the robot that is perpendicular to the goal.
    // e.g. when angle_to_goal = 90 deg, all of vx contributes to tangential velocity.
    double perpendicular_velocity = chassis_speeds.vxMetersPerSecond * angle_to_goal.getSin();

    // v = r * omega
    // We negate the value because the turret needs to spin in the opposite direction.
    double tangential_component = -(perpendicular_velocity / distance_to_goal);

    // Sum together angular and tangential components; then return velocity.
    return angular_component + tangential_component;
  }

  public enum DrivetrainState {
    PATH_FOLLOW, // Traversing a trajectory.
    MANUAL       // Teleoperated control with Xbox Controller.
  }

  public enum ClimbState {
    CLIMB_L4,     // Climb to L4.
    READY_FOR_L4, // Ready for L4 climb.
    CLIMB_L3,     // Climb to L3.
    READY_FOR_L3, // Ready for L3 climb.
    CLIMB_L2,     // Climb to L2.
    READY_FOR_L2, // Ready for L2 climb.
    STOW          // Both climb arms at min height and ratchet engaged.
  }

  public enum ScoringState {
    AUTOMATIC_HIGH_GOAL, // Shooting into the high goal, with all parameters decided automatically.
    MANUAL_HIGH_GOAL,    // Shooting into the high goal, with manually specified parameters.
    LOW_GOAL,            // Shooting into the low goal.
    AUTOMATIC_TRACK,     // Tracking the goal, with all parameters decided automatically.
    MANUAL_TRACK,        // Tracking the goal, with manually specified parameters.
    STOW                 // All scoring mechanisms stowed and out of the way.
  }

  public enum CollectionState {
    FEED,    // Cargo moving into the scoring mechanism.
    COLLECT, // Cargo moving through the feeder.
    INTAKE,  // Intake running to attempt to collect cargo.
    IDLE,    // Intake not running.
    RETRACT  // Intake retracted.
  }

  /**
   * Contains constants for all superstructure calculations.
   */
  public static class Constants {
    // Vision
    public static final double kTargetHeight = 2.6416;

    // Goal
    public static final Translation2d kGoalLocation = new Translation2d(8.2296, 4.1148);
  }
}
