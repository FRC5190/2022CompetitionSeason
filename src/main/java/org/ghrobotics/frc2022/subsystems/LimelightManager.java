package org.ghrobotics.frc2022.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.ghrobotics.frc2022.RobotState;
import org.ghrobotics.frc2022.vision.Limelight;

public class LimelightManager extends SubsystemBase {
  // Robot State
  private final RobotState robot_state_;

  // Limelight
  private final Limelight limelight_;

  // Targets
  private boolean tracking_target_ = false;

  // Alive Tracker
  private final LinearFilter alive_filter_;
  private boolean is_alive_ = false;

  /**
   * Constructs an instance of the Limelight Manager subsystem. Only one instance of this
   * subsystem should be created in the main Robot class and references to this instance should
   * be passed around the robot code.
   */
  public LimelightManager(RobotState robot_state) {
    // Store reference to robot state.
    robot_state_ = robot_state;

    // Initialize Limelight.
    limelight_ = new Limelight(Constants.kLimelightId);

    // Initialize alive tracker.
    alive_filter_ = LinearFilter.movingAverage(Constants.kAliveFilterTaps);
  }

  /**
   * This method runs periodically every 20 ms. Here, Limelight values are read and vision data
   * is sent to the robot state instance to be incorporated into the pose estimator.
   */
  @Override
  public void periodic() {
    // Update Limelight values.
    limelight_.periodic();

    // Update alive tracker from moving average.
    double latency = limelight_.getLatency();
    is_alive_ = alive_filter_.calculate(latency) > 11;

    // Check whether the Limelight is tracking targets.
    tracking_target_ = limelight_.hasTarget();

    // Check whether we have a target.
    if (tracking_target_) {
      // Get timestamp of capture (convert latency to seconds).
      double timestamp = Timer.getFPGATimestamp() - latency / 1000;

      // Get robot pose, turret angle, and hood angle at capture time.
      Pose2d robot_pose = robot_state_.getRobotPose(timestamp);
      Rotation2d turret_angle = robot_state_.getTurretAngle(timestamp);
      Rotation2d hood_angle = robot_state_.getHoodAngle(timestamp);

      // Get tx and ty values.
      double tx = limelight_.getTx();
      double ty = limelight_.getTy();

      // Calculate camera distance (x) and height (z) relative to turret at capture time.
      double turret_to_camera_x = Constants.kTurretToCameraDistance * hood_angle.getCos();
      double turret_to_camera_z = Constants.kTurretToCameraDistance * hood_angle.getSin();

      // Calculate camera height and angle from / relative to ground.
      double camera_height = Constants.kTurretHeight + turret_to_camera_z;
      double camera_angle = Math.PI / 2 - Constants.kHoodToCameraOffset - hood_angle.getRadians();

      // NOTE: ALL FURTHER GOAL CALCULATIONS ARE TO THE TAPE. "GOAL" = TAPE.
      // Calculate angle to goal. We negative value because tx is clockwise-positive whereas we
      // want the opposite.
      double angle_to_goal = -Math.toRadians(tx);

      // Calculate distance to goal.
      double distance_to_goal = (Constants.kGoalHeight - camera_height) /
          Math.tan(camera_angle + Math.toRadians(ty)) / Math.cos(angle_to_goal);

      // Calculate camera to goal transformation.
      Transform2d camera_to_goal = new Transform2d(
          new Translation2d(distance_to_goal * Math.cos(angle_to_goal),
              distance_to_goal * Math.sin(angle_to_goal)),

          // The goal is a circle, so the "rotation" of the target relative to the camera is the
          // same as the yaw to it.
          new Rotation2d(angle_to_goal));

      // Calculate turret to camera transform.
      Transform2d turret_to_camera = new Transform2d(
          new Translation2d(-turret_to_camera_x, 0), new Rotation2d());

      // Calculate robot to turret transform.
      Transform2d robot_to_turret = new Transform2d(
          new Translation2d(Constants.kRobotToTurretDistance, 0), turret_angle);

      // Calculate overall transform.
      Transform2d robot_to_goal = robot_to_turret.plus(turret_to_camera).plus(camera_to_goal)
          .plus(Constants.kGoalToGoalCenter);

      // Calculate robot pose from vision measurements.
      Rotation2d goal_rotation = robot_pose.transformBy(robot_to_goal).getRotation();
      Pose2d vision_robot_pose = new Pose2d(Constants.kGoal, goal_rotation)
          .transformBy(robot_to_goal.inverse());

      // Add to pose estimator.
      robot_state_.addVisionMeasurement(timestamp, vision_robot_pose);
    }
  }

  /**
   * Returns whether the Limelight is alive.
   *
   * @return Whether the Limelight is alive.
   */
  public boolean isLimelightAlive() {
    return is_alive_;
  }

  /**
   * Returns whether we are currently tracking targets.
   *
   * @return Whether we are currently tracking targets.
   */
  public boolean isTrackingTargets() {
    return tracking_target_;
  }

  public static class Constants {
    // IDs
    public static final String kLimelightId = "limelight";

    // Alive Tracker
    public static final int kAliveFilterTaps = 10;

    // Goal Measurements
    public static final double kGoalHeight = Units.inchesToMeters(100);
    public static final Translation2d kGoal = new Translation2d(Units.feetToMeters(27),
        Units.feetToMeters(13.5));
    public static final Transform2d kGoalToGoalCenter = new Transform2d(
        new Translation2d(Units.inchesToMeters(27), 0), new Rotation2d());

    // Robot Measurements
    public static final double kHoodToCameraOffset = Math.toRadians(30);
    public static final double kTurretToCameraDistance = Units.inchesToMeters(12);
    public static final double kTurretHeight = Units.inchesToMeters(40);
    public static final double kRobotToTurretDistance = Units.inchesToMeters(8);
  }
}
