package org.ghrobotics.frc2022;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.Objects;
import org.ghrobotics.lib.estimator.PoseEstimator;
import org.ghrobotics.lib.telemetry.MissionControl;

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

  // Turret Safety
  private boolean turret_safe_to_turn_ = false;

  // Last Vision Pose
  private Pose2d last_vision_pose_ = new Pose2d();

  // Alliance Color
  private DriverStation.Alliance alliance_ = DriverStation.Alliance.Invalid;

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

    // Add telemetry.
    MissionControl.addDouble("robot_state/x", () -> getRobotPose().getX());
    MissionControl.addDouble("robot_state/y", () -> getRobotPose().getY());
    MissionControl.addDouble("robot_state/theta", () -> getRobotPose().getRotation().getRadians());
    MissionControl.addDouble("robot_state/goal_distance",
        () -> getRobotPose().getTranslation().getDistance(Arena.kGoal));
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
   * Adds a vision measurement to the pose estimator.
   *
   * @param timestamp         The time of capture of the frame.
   * @param vision_robot_pose The robot pose as calculated from vision at the time of capture.
   */
  public void addVisionMeasurement(double timestamp, Pose2d vision_robot_pose) {
    if (Robot.kUsePoseEstimator && DriverStation.isEnabled()) {
      if (Robot.kUsePoseEstimatorInAuto || !DriverStation.isAutonomous()) {
        // Apply odometry transform from timestamp to now to vision robot pose.
        vision_robot_pose = vision_robot_pose.plus(getRobotPose().minus(getRobotPose(timestamp)));

        last_vision_pose_ = vision_robot_pose;

        // Add to pose estimator if within the tolerance.
        if (vision_robot_pose.getTranslation().getDistance(getRobotPose().getTranslation()) <
            Constants.kErrorTolerance) {
          pose_estimator_.addVisionMeasurement(vision_robot_pose);
        }
      }
    }
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
   * Updates the hood angle buffer with new measurements.
   *
   * @param angle The hood angle at the current time (measured from the horizontal).
   */
  public void updateHoodAngle(Rotation2d angle) {
    hood_buffer_.addSample(Timer.getFPGATimestamp(), angle);
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

    // Reset pose buffer.
    pose_buffer_.clear();
    pose_buffer_.addSample(Timer.getFPGATimestamp(), pose);
  }

  /**
   * Resets the robot position assuming that the robot is currently at (or close to) the fender.
   * Because the robot's gyro is very accurate, it uses it to determine which of the four
   * possible fender locations the robot is at.
   */
  public void resetPositionFromFender() {
    // Get the current robot pose.
    Pose2d robot_pose = getRobotPose();

    // Find the pose to reset to (by taking the minimum of the difference in current angle with
    // the 4 fender poses).
    Pose2d min_pose = Constants.kFenderPoses[0];
    double min_angle = 2 * Math.PI;

    for (Pose2d p : Constants.kFenderPoses) {
      double angle = p.getRotation().minus(robot_pose.getRotation()).getRadians();
      if (angle < min_angle) {
        min_angle = angle;
        min_pose = p;
      }
    }

    // Reset pose.
    resetPosition(min_pose);
  }

  /**
   * Sets the alliance color of the robot.
   *
   * @param alliance The alliance color of the robot.
   */
  public void setAlliance(DriverStation.Alliance alliance) {
    alliance_ = Objects.requireNonNullElse(alliance, DriverStation.Alliance.Invalid);
  }

  /**
   * Sets whether it is safe for the turret to turn.
   *
   * @param value Whether it is safe for the turret to turn.
   */
  public void setTurretSafeToTurn(boolean value) {
    turret_safe_to_turn_ = value;
  }

  /**
   * Returns the robot speeds at the current time.
   *
   * @return The robot speeds at the current time.
   */
  public ChassisSpeeds getRobotSpeeds() {
    return robot_speeds_;
  }

  /**
   * Returns the robot pose at the specified timestamp.
   *
   * @param timestamp The timestamp at which to get the robot pose.
   * @return The robot pose.
   */
  public Pose2d getRobotPose(double timestamp) {
    return pose_buffer_.getSample(timestamp);
  }

  /**
   * Returns the robot pose at the current time.
   *
   * @return The robot pose at the current time.
   */
  public Pose2d getRobotPose() {
    return getRobotPose(Timer.getFPGATimestamp());
  }

  /**
   * Returns the last vision robot pose.
   *
   * @return The last vision robot pose.
   */
  public Pose2d getLastVisionPose() {
    return last_vision_pose_;
  }

  /**
   * Returns the turret angle at the specified timestamp.
   *
   * @param timestamp The timestamp at which to get the turret angle.
   * @return The turret angle.
   */
  public Rotation2d getTurretAngle(double timestamp) {
    return turret_buffer_.getSample(timestamp);
  }

  /**
   * Returns the turret angle at the current time.
   *
   * @return The turret angle at the current time.
   */
  public Rotation2d getTurretAngle() {
    return getTurretAngle(Timer.getFPGATimestamp());
  }

  /**
   * Returns the hood angle at the specified timestamp.
   *
   * @param timestamp The timestamp at which to get the hood angle.
   * @return The hood angle.
   */
  public Rotation2d getHoodAngle(double timestamp) {
    return hood_buffer_.getSample(timestamp);
  }

  /**
   * Returns the hood angle at the current time.
   *
   * @return The hood angle at the current time.
   */
  public Rotation2d getHoodAngle() {
    return getHoodAngle(Timer.getFPGATimestamp());
  }

  /**
   * Returns the alliance that the robot is on.
   *
   * @return The alliance that the robot is on.
   */
  public DriverStation.Alliance getAlliance() {
    return alliance_;
  }

  /**
   * Returns whether it is safe for the turret to turn.
   *
   * @return Whether it is safe for the turret to turn.
   */
  public boolean isTurretSafeToTurn() {
    return turret_safe_to_turn_;
  }

  public static class Constants {
    // Pose Estimator
    public static final Matrix<N5, N1> kEstimatorStateStdDevs =
        VecBuilder.fill(0.02, 0.02, 0.01, 0.05, 0.05);
    public static final Matrix<N3, N1> kEstimatorLocalStdDevs =
        VecBuilder.fill(0.01, 0.01, 0.01);
    public static final Matrix<N3, N1> kEstimatorVisionStdDevs =
        VecBuilder.fill(0.05, 0.05, 0.01);

    public static final double kErrorTolerance = 1.5;

    // Buffer
    public static final double kBufferLifetime = 1.25;

    // Fender Poses
    public static final Pose2d[] kFenderPoses = new Pose2d[]{
        new Pose2d(7.759, 2.862, Rotation2d.fromDegrees(249)),
        new Pose2d(6.983, 4.664, Rotation2d.fromDegrees(158)),
        new Pose2d(9.549, 3.663, Rotation2d.fromDegrees(340)),
        new Pose2d(8.736, 5.428, Rotation2d.fromDegrees(71))
    };
  }
}
