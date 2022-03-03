package org.ghrobotics.lib;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.AngleStatistics;
import edu.wpi.first.math.estimator.UnscentedKalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.BiConsumer;

/**
 * This class is a version of {@link edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator}
 * except that vision measurements are not compensated for latency to save CPU resources.
 */
public class PoseEstimator {
  // Filter and Correction for Vision Measurements
  private final UnscentedKalmanFilter<N5, N3, N3> observer_;
  private final BiConsumer<Matrix<N3, N1>, Matrix<N3, N1>> vision_correct_;

  // Covariance Matrix for Vision Measurements
  private Matrix<N3, N3> vision_cont_R_;

  // Offsets
  private double prev_time_ = -1;
  private Rotation2d prev_angle_;
  private Matrix<N3, N1> prev_u_;
  private Rotation2d gyro_offset_;

  /**
   * Constructs an instance of the pose estimator.
   *
   * @param gyro                        The gyro angle.
   * @param initial_pose                The initial pose.
   * @param state_std_devs              The standard deviations of model states. The vector
   *                                    should be in the form [x, y, theta, dist_l, dist_r].
   * @param local_measurement_std_devs  The standard deviations of local measurements. The vector
   *                                    should be in the form [dist_l, dist_r, theta].
   * @param vision_measurement_std_devs The standard deviations of vision measurements. The
   *                                    vector should be in the form [x, y, theta].
   */
  public PoseEstimator(Rotation2d gyro, Pose2d initial_pose,
                       Matrix<N5, N1> state_std_devs, Matrix<N3, N1> local_measurement_std_devs,
                       Matrix<N3, N1> vision_measurement_std_devs) {
    // Create observer.
    observer_ = new UnscentedKalmanFilter<>(Nat.N5(), Nat.N3(),
        this::calculateXDot,
        (x, u) -> VecBuilder.fill(x.get(3, 0), x.get(4, 0), x.get(2, 0)),
        state_std_devs,
        local_measurement_std_devs,
        AngleStatistics.angleMean(2),
        AngleStatistics.angleMean(2),
        AngleStatistics.angleResidual(2),
        AngleStatistics.angleResidual(2),
        AngleStatistics.angleAdd(2),
        0.02);

    // Create vision measurement covariance matrix.
    setVisionMeasurementStdDevs(vision_measurement_std_devs);

    // Create correct() function for vision measurements.
    vision_correct_ = (u, y) -> observer_.correct(Nat.N3(),
        u, y, (x, input) -> new Matrix<>(x.getStorage().extractMatrix(0, 3, 0, 1)),
        vision_cont_R_,
        AngleStatistics.angleMean(2),
        AngleStatistics.angleResidual(2),
        AngleStatistics.angleResidual(2),
        AngleStatistics.angleAdd(2));

    // Set offsets.
    prev_angle_ = initial_pose.getRotation();
    gyro_offset_ = initial_pose.getRotation().minus(gyro);
    observer_.setXhat(createState(initial_pose, 0, 0));
  }

  /**
   * Resets the robot's position on the field. You must reset your encoders to zero when calling
   * this method.
   *
   * @param pose The robot pose.
   * @param gyro The gyro angle.
   */
  public void resetPosition(Pose2d pose, Rotation2d gyro) {
    // Reset state estimate.
    observer_.reset();
    observer_.setXhat(createState(pose, 0, 0));

    // Set offsets.
    prev_angle_ = pose.getRotation();
    gyro_offset_ = pose.getRotation().minus(gyro);
  }

  /**
   * Sets the standard deviations for vision measurements. Increase these to rely less on vision
   * measurements.
   *
   * @param vision_measurement_std_devs The standard deviations for vision measurements. The
   *                                    vector is in the form [x, y, theta].
   */
  public void setVisionMeasurementStdDevs(Matrix<N3, N1> vision_measurement_std_devs) {
    vision_cont_R_ = StateSpaceUtil.makeCovarianceMatrix(Nat.N3(), vision_measurement_std_devs);
  }

  /**
   * Adds a vision measurement as reported by the camera. Ideally, this measurement should be as
   * up-to-date as possible.
   *
   * @param vision_robot_pose The robot pose as measured by the vision system.
   */
  public void addVisionMeasurement(Pose2d vision_robot_pose) {
    // If there is no previous input, throw out the measurement.
    if (prev_u_ != null)
      vision_correct_.accept(prev_u_, StateSpaceUtil.poseTo3dVector(vision_robot_pose));
  }

  public Pose2d update(Rotation2d gyro, double average_velocity,
                       double l_dist, double r_dist) {
    // Calculate dt.
    double current_time = Timer.getFPGATimestamp();
    double dt = prev_time_ >= 0 ? current_time - prev_time_ : 0.02;
    prev_time_ = current_time;

    // Calculate actual field-relative angle from offset.
    Rotation2d angle = gyro.plus(gyro_offset_);

    // Fill input vector ([v_x, v_y, omega]).
    Matrix<N3, N1> u = VecBuilder.fill(average_velocity, 0,
        angle.minus(prev_angle_).getRadians() / dt);
    prev_u_ = u;
    prev_angle_ = angle;

    // Fill measurement vector ([d_l, d_r, theta]).
    Matrix<N3, N1> y = VecBuilder.fill(l_dist, r_dist, angle.getRadians());

    // Update observer.
    observer_.predict(u, dt);
    observer_.correct(u, y);

    // Return estimated position.
    return getEstimatedPosition();
  }

  /**
   * Returns the pose of the robot at the current time as estimated by the UKF.
   *
   * @return The robot pose.
   */
  public Pose2d getEstimatedPosition() {
    return new Pose2d(observer_.getXhat(0), observer_.getXhat(1),
        new Rotation2d(observer_.getXhat(2)));
  }

  /**
   * Calculates x' given the current state x and input u.
   *
   * @param x The current state.
   * @param u The inputs to the plant.
   * @return The change in current state (or state derivative).
   */
  private Matrix<N5, N1> calculateXDot(Matrix<N5, N1> x, Matrix<N3, N1> u) {
    // Apply a rotation matrix to align with field. Note that we don't add x because Runge-Kutta
    // integration does that for us.
    double theta = x.get(2, 0);
    Matrix<N5, N5> field_relative_mat = new MatBuilder<>(Nat.N5(), Nat.N5()).fill(
        Math.cos(theta), -Math.sin(theta), 0, 0, 0,
        Math.sin(theta), +Math.cos(theta), 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1);

    // Multiply by x, y, theta, v_l, v_r and return.
    return field_relative_mat.times(VecBuilder.fill(
        u.get(0, 0), u.get(1, 0), u.get(2, 0), u.get(0, 0), u.get(1, 0)));
  }

  /**
   * Creates a state from the given pose and distances.
   *
   * @param pose   The robot pose.
   * @param l_dist The left distance traveled (in meters).
   * @param r_dist The right distance traveled (in meters).
   * @return The system state.
   */
  private static Matrix<N5, N1> createState(Pose2d pose, double l_dist, double r_dist) {
    return VecBuilder.fill(
        pose.getX(), pose.getY(), pose.getRotation().getRadians(), l_dist, r_dist);
  }
}
