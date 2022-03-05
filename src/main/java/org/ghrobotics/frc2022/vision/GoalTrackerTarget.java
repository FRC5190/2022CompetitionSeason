package org.ghrobotics.frc2022.vision;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;

/**
 * Represents an individual target that is tracked by the goal tracker. The poses that are added
 * to each target are averaged out to avoid rapid changes.
 */
public class GoalTrackerTarget {
  // Samples within this target.
  private final List<Sample> samples_;

  // The average pose.
  private Pose2d average_pose_;

  // Filters to average out target.
  private final MedianFilter x_filter_;
  private final MedianFilter y_filter_;
  private final MedianFilter theta_filter_;

  // Whether the target is alive.
  private boolean is_alive_ = true;

  // Whether the target is real -- a target is real when it has received a certain number of data
  // points in a period of time.
  private boolean is_real_ = false;

  /**
   * Creates a target with the given initial sample. This target won't be considered "real" unless
   * we get some more measurements soon.
   *
   * @param initial_sample The initial sample with which to start off this target.
   */
  public GoalTrackerTarget(Sample initial_sample) {
    // Initialize average pose.
    average_pose_ = initial_sample.pose;

    // Initialize list of samples.
    samples_ = new ArrayList<>();
    samples_.add(initial_sample);

    // Initialize filters.
    x_filter_ = new MedianFilter(Constants.kFilterSize);
    y_filter_ = new MedianFilter(Constants.kFilterSize);
    theta_filter_ = new MedianFilter(Constants.kFilterSize);
  }

  /**
   * Adds a sample to the list.
   *
   * @param sample The sample to add.
   */
  public void addSample(Sample sample) {
    samples_.add(sample);
  }

  /**
   * Updates the target; this removes samples that are older than the max tracking lifetime and
   * recalculates the average pose based on the remaining samples.
   */
  public void update() {
    // Get current time.
    double time = Timer.getFPGATimestamp();

    // Remove expired samples.
    samples_.removeIf((sample) -> time - sample.timestamp >= Constants.kMaxTrackingLifetime);

    // Update state.
    is_alive_ = !samples_.isEmpty();
    is_real_ = samples_.size() > Constants.kRealThreshold;

    // Reset the filters.
    x_filter_.reset();
    y_filter_.reset();
    theta_filter_.reset();

    // Calculate new pose average.
    double x = 0, y = 0, theta = 0;
    for (Sample sample : samples_) {
      x = x_filter_.calculate(sample.pose.getX());
      y = y_filter_.calculate(sample.pose.getY());
      theta = theta_filter_.calculate(sample.pose.getRotation().getRadians());
    }

    // Store new pose average.
    average_pose_ = new Pose2d(x, y, new Rotation2d(theta));
  }

  /**
   * Returns the average pose of this target.
   *
   * @return The average pose of this target.
   */
  public Pose2d getPose() {
    return average_pose_;
  }

  /**
   * Returns whether this target is alive.
   *
   * @return Whether this target is alive.
   */
  public boolean isAlive() {
    return is_alive_;
  }

  /**
   * Returns whether this target is real.
   *
   * @return Whether this target is real.
   */
  public boolean isReal() {
    return is_real_;
  }

  public static class Sample {
    public double timestamp;
    public Pose2d pose;

    public Sample(double timestamp, Pose2d pose) {
      this.timestamp = timestamp;
      this.pose = pose;
    }
  }

  public static class Constants {
    // Filters
    public static final int kFilterSize = 10;

    // Lifetimes
    public static final double kMaxTrackingLifetime = 2.0;

    // Real Threshold
    public static final int kRealThreshold = 4;
  }
}
