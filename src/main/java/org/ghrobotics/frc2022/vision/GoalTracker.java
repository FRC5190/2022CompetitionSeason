package org.ghrobotics.frc2022.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;

/**
 * Used to track the various locations of targets around the field. The tracker can maintain
 * several targets at once and pick the "best" one based on several heuristics for the robot to
 * aim toward.
 */
public class GoalTracker {
  // List of targets that we are currently tracking.
  private final List<GoalTrackerTarget> targets_;

  public GoalTracker() {
    // Initialize list of targets.
    targets_ = new ArrayList<>();
  }

  /**
   * Adds samples to the goal tracker.
   *
   * @param timestamp The time of capture of the poses.
   * @param poses     The poses to add.
   */
  public void addSamples(double timestamp, Pose2d... poses) {
    // Discard samples from the future.
    if (timestamp > Timer.getFPGATimestamp())
      return;

    // Find which target we need to add this sample to.
    for (Pose2d pose : poses) {
      // If there are no targets, add one and continue.
      if (targets_.isEmpty()) {
        targets_.add(new GoalTrackerTarget(new GoalTrackerTarget.Sample(timestamp, pose)));
        continue;
      }

      // Find the closest target to the current pose.
      GoalTrackerTarget closest_target = targets_.size() < 2 ? targets_.get(0) :
          targets_.stream().min((t1, t2) -> {
            double t1_dist = t1.getPose().getTranslation().getDistance(pose.getTranslation());
            double t2_dist = t2.getPose().getTranslation().getDistance(pose.getTranslation());
            return Double.compare(t1_dist, t2_dist);
          }).get();

      if (closest_target.getPose().getTranslation().getDistance(pose.getTranslation()) <=
          Constants.kSameTargetTolerance) {
        // Add this sample to that existing target.
        closest_target.addSample(new GoalTrackerTarget.Sample(timestamp, pose));
      } else {
        // If not, create a new target.
        targets_.add(new GoalTrackerTarget(new GoalTrackerTarget.Sample(timestamp, pose)));
      }
    }
  }

  /**
   * Updates the goal tracker. Here, all targets are updated and old ones are removed from the
   * buffer, except when a minimum keep-alive target count is specified.
   */
  public void update() {
    // Get the current time.
    double time = Timer.getFPGATimestamp();

    // Update and remove old targets.
    targets_.removeIf((target) -> {
      target.update();
      return !target.isAlive();
    });
  }

  /**
   * Returns the closest target to the provided robot pose.
   *
   * @param robot_pose The robot pose.
   * @return The closest target to the provided robot pose.
   */
  public Pose2d getClosestTarget(Pose2d robot_pose) {
    // If the target is empty, just point to origin.
    if (targets_.isEmpty())
      return new Pose2d();

    return targets_.stream().min((t1, t2) -> {
      double t1_dist = t1.getPose().getTranslation().getDistance(robot_pose.getTranslation());
      double t2_dist = t2.getPose().getTranslation().getDistance(robot_pose.getTranslation());
      return Double.compare(t1_dist, t2_dist);
    }).get().getPose();
  }

  public static class Constants {
    // Tolerances
    public static final double kSameTargetTolerance = Units.inchesToMeters(12);
  }
}
