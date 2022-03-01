package org.ghrobotics.frc2022;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;

public class RobotState {
  // Pose Estimator
  private final DifferentialDrivePoseEstimator pose_estimator_;

  // Robot Speeds
  private ChassisSpeeds robot_speeds_ = new ChassisSpeeds(0, 0, 0);

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
    pose_estimator_ = new DifferentialDrivePoseEstimator(new Rotation2d(), new Pose2d(),
        Constants.kEstimatorStateStdDevs, Constants.kEstimatorLocalStdDevs,
        Constants.kEstimatorVisionStdDevs);
  }

  /**
   * Updates the pose estimator with measurements from encoders and gyro.
   *
   * @param l_position The measured left encoder position in meters.
   * @param r_position The measured right encoder position in meters.
   * @param l_velocity The measured left encoder velocity in meters per second.
   * @param r_velocity The measured right encoder velocity in meters per second.
   * @param angle      The measured heading of the robot.
   */
  public void updateRobotPose(double l_position, double r_position, double l_velocity,
                              double r_velocity, Rotation2d angle) {
    // Create wheel speeds object.
    DifferentialDriveWheelSpeeds wheel_speeds =
        new DifferentialDriveWheelSpeeds(l_velocity, r_velocity);

    // Store encoder and gyro values.
    l_encoder_ = l_position;
    r_encoder_ = r_position;
    gyro_ = angle;

    // Update the pose estimator with local measurements.
    pose_estimator_.update(angle.minus(gyro_offset_), wheel_speeds,
        l_position - l_encoder_offset_, r_position - r_encoder_offset_);
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
  }
}
