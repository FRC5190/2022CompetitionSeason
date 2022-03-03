package org.ghrobotics.frc2022;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import org.ghrobotics.lib.PoseEstimator;

public class RobotState {
  // Pose Estimator
  private final PoseEstimator pose_estimator_;

  // Robot Speeds
  private ChassisSpeeds robot_speeds_ = new ChassisSpeeds(0, 0, 0);

  // Buffers
  private final TimeInterpolatableBuffer<Pose2d> pose_buffer_;
  private final TimeInterpolatableBuffer<Rotation2d> turret_buffer_;
  private final TimeInterpolatableBuffer<Rotation2d> hood_buffer_;

  // Sensor Offsets
  private double l_encoder_offset_ = 0.0;
  private double r_encoder_offset_ = 0.0;
  private Rotation2d gyro_offset_ = new Rotation2d();

  private double l_encoder_ = 0.0;
  private double r_encoder_ = 0.0;
  private Rotation2d gyro_ = new Rotation2d();

  /**
   * Constructs a "robot state" instance. This keeps track of various states on the robot,
   * including robot pose, turret angle, and hood angle across time.
   */
  public RobotState() {
    // Initialize pose estimator.
    pose_estimator_ = new PoseEstimator(new Rotation2d(), new Pose2d(),
        Constants.kEstimatorStateStdDevs, Constants.kEstimatorLocalStdDevs,
        Constants.kEstimatorVisionStdDevs);

    // Initialize buffers.
    pose_buffer_ = TimeInterpolatableBuffer.createBuffer(Constants.kBufferLifetime);
    turret_buffer_ = TimeInterpolatableBuffer.createBuffer(Constants.kBufferLifetime);
    hood_buffer_ = TimeInterpolatableBuffer.createBuffer(Constants.kBufferLifetime);
  }

  /**
   * Updates the pose estimator with measurements from encoders and gyro.
   *
   * @param l_position       The measured left encoder position in meters.
   * @param r_position       The measured right encoder position in meters.
   * @param average_velocity The average velocity of the drivetrain in meters per second.
   * @param angle            The measured heading of the robot.
   */
  public void updateRobotPose(double l_position, double r_position,
                              double average_velocity, Rotation2d angle) {
    // Store encoder and gyro values.
    l_encoder_ = l_position;
    r_encoder_ = r_position;
    gyro_ = angle;

    // Update the pose estimator with local measurements.
    Pose2d pose = pose_estimator_.update(angle.minus(gyro_offset_), average_velocity,
        l_position - l_encoder_offset_, r_position - r_encoder_offset_);

    // Add pose to buffer.
    pose_buffer_.addSample(Timer.getFPGATimestamp(), pose);
  }

  /**
   * Updates the robot speeds.
   *
   * @param speeds The current robot speeds, as measured by the encoders and gyro.
   */
  public void updateRobotSpeeds(ChassisSpeeds speeds) {
    // Update local variable.
    robot_speeds_ = speeds;
  }

  /**
   * Updates the turret angle buffer with new measurements.
   *
   * @param angle The turret angle at the current time.
   */
  public void updateTurretAngle(Rotation2d angle) {
    turret_buffer_.addSample(Timer.getFPGATimestamp(), angle);
  }

  /**
   * Updates the hood angle buffer with new measurements.
   *
   * @param angle The hood angle at the current time (measured from the horizontal).
   */
  public void updateHoodAngle(Rotation2d angle) {
    hood_buffer_.addSample(Timer.getFPGATimestamp(), angle);
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param timestamp The time of capture of the image.
   * @param tx        Yaw angle to the goal (in degrees, cw positive).
   * @param ty        Pitch angle to the goal (in degrees, upward positive).
   */
  public void addVisionMeasurement(double timestamp, double tx, double ty) {
    // Get turret and hood angle at capture time.
    Rotation2d turret_angle = turret_buffer_.getSample(timestamp);
    Rotation2d hood_angle = hood_buffer_.getSample(timestamp);

    // Get robot pose at capture time.
    Pose2d capture_pose = pose_buffer_.getSample(timestamp);

    // Calculate camera distance (x) and height (z) relative to turret at capture time.
    double turret_to_camera_x = Constants.kTurretToCameraDistance * hood_angle.getCos();
    double turret_to_camera_z = Constants.kTurretToCameraDistance * hood_angle.getSin();

    // Calculate camera height and angle from / relative to ground.
    double camera_height = Constants.kTurretHeight + turret_to_camera_z;
    double camera_angle = Math.PI / 2 - (Constants.kHoodToCameraOffset + hood_angle.getRadians());

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

    // Create transform chain.
    Transform2d transform_chain = robot_to_turret.plus(turret_to_camera).plus(camera_to_goal);

    // Calculate the field-relative rotation of the goal. Here, we have to rely on the robot's gyro.
    // This should also be equal to capture_pose.rotation + turret.rotation + camera_to_goal
    // .rotation.
    Rotation2d goal_rotation = capture_pose.transformBy(transform_chain).getRotation();

    // Using the goal rotation and distance to outer rim, calculate robot pose at capture time.
    Pose2d capture_vision_pose = new Pose2d(Constants.kGoal, goal_rotation).transformBy(
        new Transform2d(new Translation2d(-Constants.kGoalDistanceToOuterRim, 0),
            new Rotation2d())).transformBy(transform_chain.inverse());

    // Calculate vision pose at current time using difference in odometry between capture time
    // and current time. This odometry difference should be very accurate because it's only on
    // the order of a few milliseconds.
    // vision_pose_now = vision_pose_then + (odometry_pose_now - odometry_pose_then).
    Pose2d vision_pose_now = capture_vision_pose.plus(getRobotPose().minus(capture_pose));

    // Add to pose estimator.
    pose_estimator_.addVisionMeasurement(vision_pose_now);
  }

  /**
   * Resets the position of the robot.
   *
   * @param pose The position to reset to.
   */
  public void resetPosition(Pose2d pose) {
    // Store offsets.
    l_encoder_offset_ = l_encoder_;
    r_encoder_offset_ = r_encoder_;
    gyro_offset_ = gyro_;

    // Reset pose estimator.
    pose_estimator_.resetPosition(pose, new Rotation2d());
  }

  /**
   * Returns the robot pose at the current time.
   *
   * @return The robot pose at the current time.
   */
  public Pose2d getRobotPose() {
    return pose_estimator_.getEstimatedPosition();
  }

  public static class Constants {
    // Pose Estimator
    public static final Matrix<N5, N1> kEstimatorStateStdDevs =
        VecBuilder.fill(0.02, 0.02, 0.01, 0.05, 0.05);
    public static final Matrix<N3, N1> kEstimatorLocalStdDevs =
        VecBuilder.fill(0.01, 0.01, 0.01);
    public static final Matrix<N3, N1> kEstimatorVisionStdDevs =
        VecBuilder.fill(0.1, 0.1, 0.01);

    // Buffer
    public static final double kBufferLifetime = 1.25;

    // Goal Measurements
    public static final double kGoalHeight = Units.inchesToMeters(85); // TODO
    public static final double kGoalDistanceToOuterRim = Units.inchesToMeters(24); // TODO
    public static final Translation2d kGoal =
        new Translation2d(Units.feetToMeters(27), Units.feetToMeters(13.5));

    // Robot Measurements
    public static final double kHoodToCameraOffset = Math.toRadians(30);
    public static final double kTurretToCameraDistance = Units.inchesToMeters(9); // TODO
    public static final double kTurretHeight = Units.inchesToMeters(42); // TODO
    public static final double kRobotToTurretDistance = Units.inchesToMeters(8.5); // TODO
  }
}
