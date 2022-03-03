package org.ghrobotics.frc2022.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.ghrobotics.frc2022.RobotState;
import org.ghrobotics.frc2022.vision.Limelight;

public class LimelightManager extends SubsystemBase {
  // Robot State
  private final RobotState robot_state_;

  // Limelight
  private final Limelight limelight_;

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
    is_alive_ = alive_filter_.calculate(limelight_.getLatency()) > 11;

    // Check whether we have a target.
    if (limelight_.hasTarget()) {
      // Get tx and ty values.
      double tx = limelight_.getTx();
      double ty = limelight_.getTy();

      // Add to robot state.
      robot_state_.addVisionMeasurement(Timer.getFPGATimestamp() - limelight_.getLatency() / 1000.0,
          tx, ty);
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

  public static class Constants {
    // IDs
    public static final String kLimelightId = "limelight";

    // Alive Tracker
    public static final int kAliveFilterTaps = 10;
  }
}
