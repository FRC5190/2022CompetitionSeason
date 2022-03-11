package org.ghrobotics.frc2022.commands;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.ghrobotics.frc2022.subsystems.Climber;

public class ClimbReset extends CommandBase {
  // Reference to subsystem.
  private final Climber climber_;

  // Filters for current.
  private final MedianFilter left_climb_current_filter_;
  private final MedianFilter right_climb_current_filter_;

  // Current averages.
  private double left_climb_current_ = 0;
  private double right_climb_current_ = 0;

  // Whether we are done resetting.
  private boolean left_reset_complete_ = false;
  private boolean right_reset_complete_ = false;

  // Timer to make sure filter fills up.
  private final Timer timer_;

  /**
   * Resets the state of the climber, bringing both arms down until the limit.
   *
   * @param climber Reference to climber subsystem.
   */
  public ClimbReset(Climber climber) {
    // Assign member variables.
    climber_ = climber;

    // Initialize filters.
    left_climb_current_filter_ = new MedianFilter(Constants.kFilterSize);
    right_climb_current_filter_ = new MedianFilter(Constants.kFilterSize);

    // Initialize timer.
    timer_ = new Timer();

    // Set subsystem requirements.
    addRequirements(climber_);
  }

  @Override
  public void initialize() {
    // Disable soft limits.
    climber_.enableSoftLimits(false);

    // Undo brake.
    climber_.setBrake(false);

    // Pivot both arms out.
    climber_.setPivot(true, true);

    // Start timer.
    timer_.start();
  }

  @Override
  public void execute() {
    // Check whether we should start assigning currents at this point.
    boolean assign = timer_.hasElapsed(Constants.kFilterSize * 0.02);

    // Run each climber arm down until current threshold is reached.
    if (!left_reset_complete_) {
      climber_.setLeftPercent(-0.15);
      double l = left_climb_current_filter_.calculate(climber_.getLeftSupplyCurrent());
      left_climb_current_ = assign ? l : 0;

      // Check current value.
      if (left_climb_current_ > Constants.kCurrentThreshold)
        left_reset_complete_ = true;
    } else {
      climber_.setLeftPercent(0);
    }

    if (!right_reset_complete_) {
      climber_.setRightPercent(-0.15);
      double r = right_climb_current_filter_.calculate(climber_.getRightSupplyCurrent());
      right_climb_current_ = assign ? r : 0;

      // Check current value.
      if (right_climb_current_ > Constants.kCurrentThreshold)
        right_reset_complete_ = true;
    } else {
      climber_.setRightPercent(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Re-enable soft limits and zero outputs.
    climber_.enableSoftLimits(true);
    climber_.setLeftPercent(0);
    climber_.setRightPercent(0);

    // Zero climb.
    if (!interrupted) {
      climber_.zero();
    }

    // Stop timer.
    timer_.stop();
  }

  @Override
  public boolean isFinished() {
    return left_reset_complete_ && right_reset_complete_;
  }

  public static class Constants {
    // Filter Size
    public static final int kFilterSize = 25;

    // Current Threshold
    public static final int kCurrentThreshold = 4;
  }
}
